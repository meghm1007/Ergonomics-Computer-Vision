import numpy as np
import cv2
import cvzone
import pyautogui
from cvzone.FaceMeshModule import FaceMeshDetector
import math

#Find the size of screen
size = pyautogui. size()
width = pyautogui.size()[0]
height = pyautogui.size()[1]
print(width, height)

#Get the video
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)

#Ergonomics reports
textList = ["Screen Distance", "LR Angle (tan)", "UD Angle(sin)"]

#See face
while True:
    success, img = cap.read()
    imageText = np.zeros_like(img)
    img, faces = detector.findFaceMesh(img)

    if faces:
        #Detect the face, left, right eye
        face = faces[0]
        pointLeft = face[145]
        pointRight = face[374]

        #Mark the eyes and a line
        cv2.circle(img, pointLeft, 5, (255,0,255), cv2.FILLED)
        cv2.circle(img, pointRight, 5, (255, 0, 255), cv2.FILLED)
        cv2.line(img, pointLeft, pointRight,(0,200,0), 2)

        #Measure the distance
        w, _ = detector.findDistance(pointLeft, pointRight)

        #Finding the focal length
        W = 6.3 #avg value for male and female
        f = 840
        #Calculate the distance in cm
        d = (W*f)/w

        #Calculate the adequate distance between screen and eyes
        if d<51:
            textList[0] = "Move Back.\nYou're too close to the screen"
        else:
            textList[0] = "Screen-eye distance is good"

        cvzone.putTextRect(img, f'{int(d)}cm', (face[10][0] - 75, face[10][1] - 50), scale=2)

        #Calculate angle (for left and right left)
        dx = pointRight[0] - pointLeft[0]
        dy = pointRight[1] - pointLeft[1]

        angleLR = math.degrees(math.atan2(dy, dx))
        angleLR = round(angleLR,1)

        #Display message according to the angle
        if angleLR<=6 and angleLR>=-6:
            textList[1] = "Good LR Angle maintained"
        elif angleLR>6:
            textList[1] = "Turn your head towards the left"
        elif angleLR<-6:
            textList[1] = "Turn your head towards the right"

        print(angleLR)

        #Calculate the up and down angle
        left_eye_y = pointLeft[1]
        right_eye_y = pointRight[1]

        dy = right_eye_y - left_eye_y

        # Adjust the angles to achieve the desired behavior
        angleUD = math.degrees(math.asin(dy / 100))

        if angleUD>-2 and angleUD<2:
            textList[2] = "Good UD Angle maintained"

        elif angleUD>2:
            textList[2] = "Move head down"

        elif angleUD<-2:
            textList[2] = "Move head up"

        for i,text in enumerate(textList):
            singleHeight = 20 + int(d/5)
            scale = 0.2 + d/100
            cv2.putText(imageText, text, (50,50+(i*singleHeight)), cv2.FONT_ITALIC, scale, (255,255,255), 1)
            cv2.putText(img, f"Angle(LR): {int(angleLR)}, Angle(UD): {int(angleUD)}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    imageStacked = cvzone.stackImages([img, imageText],2,1)


    cv2.imshow("Image", imageStacked)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()