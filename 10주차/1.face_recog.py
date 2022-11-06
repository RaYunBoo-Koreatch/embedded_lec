import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

cap = cv2.VideoCapture(0, cv2.CAP_V4L)  # 첫번째 카메라 영상
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 카메라 영상 넓이
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # 카메라 영상 높이

while (True):
    ret, img = cap.read()
    img = cv2.flip(img, -1)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.2, 5)
    print("Number of faces detected: " + str(len(faces)))

    for (x, y, w, h) in faces:
        img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)
    
    if len(faces) > 0:
        cv2.imwrite("imgs/face.png", img)

    cv2.imshow('img', img)

    k = cv2.waitKey(30) & 0xff
    if k == 27:  # Esc 키를 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()
