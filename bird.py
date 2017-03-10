import numpy as np
import cv2

def get_center():
    min = video_width - (video_width * center_threshold)
    max = video_width + (video_width * center_threshold)
    return min, max

video = cv2.VideoCapture('red_bird.mp4')
# video = cv2.VideoCapture('test1.mp4')
# video = cv2.VideoCapture('test3.mp4')

video_width = int(video.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
video_height = int(video.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))

# defines how much is considered center
center_threshold = 0.05
center_min, center_max = get_center()

# take first frame of the video
ret,frame = video.read()

# setup initial location of window
# yPos,height,xPos,width - region of image
<<<<<<< HEAD
yPos,height,xPos,width = video_height/2,20,video_width/2,20
=======
yPos,height,xPos,width = 110,50,620,20
>>>>>>> 48918dae98b24f1b09e949489ce3d8cd563893ea
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

def get_direction(x):
    print(x, center_min, center_max)
    if (center_min < x < center_max):
        return 'center'
    elif (x < center_min):
        return 'left'
    else:
        return 'right'


while(1):
    ret ,frame = video.read()
    # frame = cv2.resize(frame, (0,0), fx=0.3, fy=0.3)
    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[2],histograma,[0,180],1)

        # ======= MEAN SHIFT EXAMPLE =======
        ret, track_window = cv2.meanShift(dst, track_window, term_crit)
        # # # Draw it on image
        x,y,w,h = track_window
        cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        cv2.imshow('img2',frame)

        status = get_direction(x)
        print(status)
        print('')

        # ======= CAM SHIFT EXAMPLE =======
        # ret, track_window = cv2.CamShift(dst, track_window, term_crit)
        # # Draw it on image
        # pts = cv2.cv.BoxPoints(ret)
        # pts = np.int0(pts)
        # cv2.polylines(frame,[pts],True, 255,2)
        # cv2.imshow('img2',frame)

        k = cv2.waitKey(20) & 0xff
        if k == 27:
            break
        else:
            cv2.imwrite(chr(k)+".jpg",frame)

    else:
        break

cv2.destroyAllWindows()
video.release()
