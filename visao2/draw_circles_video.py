import cv2
import cv2.cv as cv
import numpy as np
from matplotlib import pyplot as plt
import time

#cap = cv2.VideoCapture('hall_box_battery_1024.mp4')
cap = cv2.VideoCapture(0)
cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)


while(True):
    # Capture frame-by-frame
    print("New frame")
    ret, frame = cap.read()
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img = frame

    print("Will apply HoughCircles")
    circles = []
    circles=cv2.HoughCircles(gray,cv.CV_HOUGH_GRADIENT,2,20,param1=50,param2=30,minRadius=3,maxRadius=100)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

    # Draw a diagonal blue line with thickness of 5 px
    # cv2.line(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
    cv2.line(frame,(0,0),(511,511),(255,0,0),5)

    # cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
    cv2.rectangle(frame,(384,0),(510,128),(0,255,0),3)

    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,'Ninjutsu ;)',(200,500), font, 2,(255,255,255),2,cv2.CV_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print("No circles were found")
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
