import numpy as np
import cv2

cap = cv2.VideoCapture('red_bird.mp4')
# cap = cv2.VideoCapture('test1.mp4')

# take first frame of the video
ret,frame = cap.read()

# setup initial location of window
# yPos,height,xPos,width - region of image
yPos,height,xPos,width = 110,50,620,20
track_window = (xPos,yPos,width,height)

# set up the ROI for tracking
roi = frame[yPos:yPos+height, xPos:xPos+width]
hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# ==========
# HISTOGRAMA DO PAPAGAIO
pic = cv2.imread("head.jpg")
picgray = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)
histograma = cv2.calcHist([picgray],[0], None,[180],[0,180])
cv2.normalize(histograma,histograma,0,255,cv2.NORM_MINMAX)

# Setup the termination criteria, either 10 iteration or move by at least 1 pt
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

while(1):
    ret ,frame = cap.read()
    # frame = cv2.resize(frame, (0,0), fx=0.3, fy=0.3)
    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[2],histograma,[0,180],1)

        # apply meanshift to get the new location
        #ret, track_window = cv2.meanShift(dst, track_window, term_crit)
        ret, track_window = cv2.CamShift(dst, track_window, term_crit)
        #Draw it on image
        pts = cv2.cv.BoxPoints(ret)
        pts = np.int0(pts)
        cv2.polylines(frame,[pts],True, 255,2)
        cv2.imshow('img2',frame)
        #
        # # Draw it on image
        # x,y,w,h = track_window
        # cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        # cv2.imshow('img2',frame)

        k = cv2.waitKey(20) & 0xff
        if k == 27:
            break
        else:
            cv2.imwrite(chr(k)+".jpg",frame)

    else:
        break

cv2.destroyAllWindows()
cap.release()
