import numpy as np
import cv2 as cv

cv.namedWindow('map')
map_width = 512
map_height = 512

posX = 59
posY = 256

EposX = map_width - 59
EposY = 256

while True:
    img = np.full((map_width,map_height,3),0,np.uint8)
    cv.rectangle(img, (posX-32,posY-32), (posX+32, posY+32), (255,255,255),thickness = -1) #크기 인식 우리 로봇
    cv.rectangle(img, (EposX-32,EposY-32), (EposX+32, EposY+32), (0,255,0),thickness = -1) #크기 인식 상대 로봇
    cv.circle(img, (int(map_width/2), int(map_height/2)), 75, (0,0,255), thickness = -1) #중심 위치 빨간 영역
    cv.circle(img, (0, 0), 150, (255,0,0), thickness = -1) #중심 위치 파란 영역
    cv.circle(img, (map_width,  0), 150, (255,0,0), thickness = -1) #중심 위치 파란 영역
    cv.circle(img, (0, map_height), 150, (255,0,0), thickness = -1) #중심 위치 파란 영역
    cv.circle(img, (map_width, map_height), 150, (255,0,0), thickness = -1) #중심 위치 파란 영역
    posX += 1
    posY += 1

    cv.imshow('map', img)
    if cv.waitKey(1) & 0xFF == 27:
        break

cv.destroyAllWindows()
