#include "mbed.h"
#include "MPU6050.h"
#include "Thread.h"

#define MCU_CONTROL_RATE 10
#define IMU_UPDATE_RATE 10
#define pi 3.141592654
//-----------------------------------------------------------------------------------------------------------------------------------
Thread IMU_thread(osPriorityHigh);

MPU6050 mpu6050;

float ARoll, APitch, AYaw = 0;//가속도 센서 롤, 피치, 요
float GRoll, GPitch, GYaw = 0;//자이로 센서 롤, 피치, 요
float gyroX, gyroY, gyroZ = 0;
float accelX, accelY, accelZ = 0;
float lgyroX, lgyroY, lgyroZ = 0;
float FPitch = 0;//필터 적용된 피치값
float FRoll = 0;//필터 적용된 롤값
float FYaw = 0;//필터 적용된 롤값
float velX, velY, preVelX, preVelY;
float lastVelX;
float PWMVelR, PWMVelL;
//-----------------------------------------------------------------------------------------------------------------------------------
DigitalIn button(USER_BUTTON);

RawSerial pc(USBTX, USBRX, 115200);

DigitalOut DirL(D7);
DigitalOut DirR(D4);
PwmOut pwmL(D6);
PwmOut pwmR(D5);

float t;

int mode = 1000;

Timer timer1;
 
void timerST () {
    timer1.start();  // 타이머 시작
    t = timer1.read_ms();  // 경과 시간 계산
}

void timerEND () {
    timer1.stop();  // 타이머 정지
    timer1.reset(); // 타이머 초기화
    t = 0;
}

void IMU_thread_loop(){
    
    uint64_t Now_IMU,Work_IMU;

    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68){
        pc.printf("MPU6050 is online...");
        wait(1);

        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050();
            pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

            pc.printf("\nMPU6050 passed self test... Initializing");
            wait(2);
        }
        else pc.printf("\nDevice did not the pass self-test!\n\r");
    }
    else {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }

    while(1){
        Now_IMU=rtos::Kernel::get_ms_count();

        //If data ready bit set, all data registers have new data
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
            mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu6050.readAccelData(accelCount);
            mpu6050.getGres();
            mpu6050.getAres();

            gyroY = (float)gyroCount[0]*gRes;
            gyroX = (float)gyroCount[1]*gRes;
            gyroZ = (float)gyroCount[2]*gRes; // - gyroBias[2];
            
            accelX = (float)accelCount[0]*aRes*10;
            accelY = (float)accelCount[1]*aRes*10;
            accelZ = (float)accelCount[2]*aRes*10;
        }

        ARoll = (180/pi)*(atan(accelX/(sqrt((accelY*accelY) +(accelZ*accelZ)))));
        APitch = (180/pi)*(atan(accelY/(sqrt((accelX*accelX)+(accelZ*accelZ)))));
        AYaw = (180/pi)*(atan((sqrt((accelX*accelX)+(accelY*accelY)))/accelZ));

        GYaw += ((lgyroZ+gyroZ)/2)*0.01*2;
        GPitch += ((lgyroY+gyroY)/2)*0.01*2;
        GRoll += ((lgyroX+gyroX)/2)*0.01*2;

        FPitch = (0.9*(FPitch+(((lgyroY+gyroY)/2)*2*0.01)))+0.1*APitch;//complementary fillter
        FRoll = (0.9*(FRoll+(((lgyroX+gyroX)/2)*2*0.01)))+0.1*ARoll;//complementary fillter
        FYaw = (0.9*(FYaw+(((lgyroZ+gyroZ)/2)*2*0.01)))+0.1*ARoll;

        lgyroZ = gyroZ;
        lgyroY = gyroY;
        lgyroX = gyroX;

        velX = preVelX+accelX*0.01;
        velY = preVelY+accelY*0.01;

        lastVelX = 0.9*(lastVelX+accelX*0.01) + 0.1*PWMVelR;

        preVelX = velX;
        preVelY = velY;

        pc.printf("%f, %f\n",lastVelX,velX);

        // pc.printf("velY: %f\n",velY);
        
        Work_IMU=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(IMU_UPDATE_RATE-(Work_IMU-Now_IMU)));
    }
}

double setpwmL;
double anspwmL;

void moter (int dirleft , int dirright , double pwmleft , double pwmright) {
    DirL = dirleft;
    DirR = dirright;

    if (0.2 <= pwmleft && pwmleft <= 0.8 ) {
        setpwmL = 0.125*(pwmleft*pwmleft) + 0.895*pwmleft + 0.06;
        anspwmL = round(setpwmL * 1000) / 1000.0;
    }
    else {
        anspwmL = pwmleft;
    }

    pwmL = anspwmL;
    pwmR = pwmright;

    if (pwmleft <= -0.99) {
        dirleft = 1;
        pwmleft = 0.99;
    }
    else if (pwmleft < 0 && pwmleft > -0.99) {
        dirleft = 1;
        abs(pwmleft);
    }
    else if (pwmleft >= 0.99) {
        dirleft = 0;
        pwmleft = 0.99;
    }
    else {
        dirleft = 0;
    }

    if (pwmright <= -0.99) {
        dirright = 1;
        pwmright = 0.99;
    }
    else if (pwmright < 0 && pwmright > -0.99) {
        dirright = 1;
        abs(pwmright);
    }
    else if (pwmright >= 0.99) {
        dirright = 0;
        pwmright = 0.99;
    }
    else {
        dirright = 0;
    }
}

void startMotor(){
    moter(0,0,0.3,0.3);
}

void stopMotor(){
    moter(0,0,0,0);
}

// main() runs in its own thread in the OS
int main() {
    osThreadSetPriority(osThreadGetId(),osPriorityRealtime7);

    pwmL.period_us(66); // 15KHz
    pwmR.period_us(66); // 15KHz

    uint64_t Now_M,Work_M;

    IMU_thread.start(&IMU_thread_loop);

    while(1) {
        Now_M = rtos::Kernel::get_ms_count();

        if (mode == 1000) {
            moter(0,0,0,0);
            PWMVelR = 0;
            if(button == 0) {  // 버튼 누르면 시작
                mode = 0;
            }
        }
        else if (mode == 0) {    //처음 시작 회전
            timerST();

            if (t<1000) {
                moter(0,0,0,0);
                PWMVelR = 0;
            }

            else if(1000 <= t && t < 2000) {
                moter(0, 0, 0.3, 0.3);
                PWMVelR = 0.29;
            }

            else {
                timerEND();
                mode = 1000;
            }
        }
        // if (mode == 1001) {
        //     moter(0,0,0,0);
        //     if(button == 0) {  // 버튼 누르면 시작
        //         mode = 1;
        //     }
        // }
        // else if (mode == 1) {    //처음 시작 회전
        //     timerST();

        //     if (t<1000) {
        //         moter(0,0,0,0);
        //     }

        //     else if(1000 <= t && t < 2000) {
        //         moter(0, 0, 0.3, 0.3);  
        //     }

        //     else {
        //         timerEND();
        //         mode = 1002;
        //     }
        // }
        // if (mode == 1002) {
        //     moter(0,0,0,0);
        //     if(button == 0) {  // 버튼 누르면 시작
        //         mode = 2;
        //     }
        // }
        // else if (mode == 2) {    //처음 시작 회전
        //     timerST();

        //     if (t<1000) {
        //         moter(0,0,0,0);
        //     }

        //     else if(1000 <= t && t < 2500) {
        //         moter(0, 0, 0.3, 0.3);  
        //     }

        //     else {
        //         timerEND();
        //         mode = 1003;
        //     }
        // }
        // if (mode == 1003) {
        //     moter(0,0,0,0);
        //     if(button == 0) {  // 버튼 누르면 시작
        //         mode = 3;
        //     }
        // }
        // else if (mode == 3) {    //처음 시작 회전
        //     timerST();

        //     if (t<1000) {
        //         moter(0,0,0,0);
        //     }

        //     else if(1000 <= t && t < 3000) {
        //         moter(0, 0, 0.3, 0.3);  
        //     }

        //     else {
        //         timerEND();
        //         mode = 1000;
        //     }
        // }
        Work_M=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(MCU_CONTROL_RATE-(Work_M-Now_M)));
    }
}
