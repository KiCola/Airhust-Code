import cv2
import numpy as np
import mediapipe
import time
import PoseModule as pm

cap = cv2.VideoCapture('./woman.mp4')
detector = pm.poseDetector()
count = 0
dir = 0
pTime = 0
while True:
    success, img = cap.read()
    # img = cv2.resize(img,(720,1280))
    img = detector.findPose(img,False)
    lmList = detector.findPosition(img,False)
    # print(lmList)
    if len(lmList) != 0:
        # # right arm
        # detector.findAngle(img,12,14,16)
        # left arm
        angle = detector.findAngle(img,11,13,15)
        per = np.interp(angle,(180,245),(0,100))
        # print(angle,per)

        # check for the dumbbel curls
        if per == 100:
            if dir == 0:
                count += 0.5
                dir = 1
        if per == 0:
            if dir == 1:
                count += 0.5
                dir = 0
        # print(count)
        cv2.putText(img,f'{int(count)}',(50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,0,0),5)
    cTime = time.time()
    fps = 1/(cTime-pTime)
    pTime = cTime
    cv2.putText(img,f'fps:{int(fps)}',(450,50),cv2.FONT_HERSHEY_PLAIN,3,(255,0,0),5)

    cv2.imshow('img',img)
    cv2.waitKey(1)

