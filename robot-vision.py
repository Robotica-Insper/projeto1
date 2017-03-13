# USAGE
# python track.py --video video/sample.mov

# import the necessary packages
import numpy as np
import argparse
import cv2
import rospy
from geometry_msgs.msg import Twist

# initialize the current frame of the video, along with the list of
# ROI points along with whether or not this is input mode
frame = None
roiPts = []
inputMode = False

def selectROI(event, x, y, flags, param):
    # grab the reference to the current frame, list of ROI
    # points and whether or not it is ROI selection mode
    global frame, roiPts, inputMode

    # if we are in ROI selection mode, the mouse was clicked,
    # and we do not already have four points, then update the
    # list of ROI points with the (x, y) location of the click
    # and draw the circle
    if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
        roiPts.append((x, y))
        cv2.circle(frame, (x, y), 4, (0, 255, 0), 2)
        cv2.imshow("frame", frame)


video_width, video_height = 0, 0
center_min, center_max = 0, 0
center_threshold = 0.1

default_area = 10000 # pixel density

def get_center():
    global video_width, video_height
    min = (video_width // 2) - (video_width * center_threshold)
    max = (video_width // 2) + (video_width * center_threshold)
    return min, max

def get_direction(x):
    # print(x, center_min, center_max)
    if (center_min < x < center_max):
        return 'center'
    elif (x < center_min):
        return 'left'
    else:
        return 'right'


def main():
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
        help = "path to the (optional) video file")
    args = vars(ap.parse_args())

    # grab the reference to the current frame, list of ROI
    # points and whether or not it is ROI selection mode
    global frame, roiPts, inputMode, video_width, video_height, center_min, center_max

    # if the video path was not supplied, grab the reference to the
    # camera
    if not args.get("video", False):
        camera = cv2.VideoCapture(0)

        # otherwise, load the video
    else:
        camera = cv2.VideoCapture(args["video"])


    # get video dimensions
    video_width = int(camera.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    video_height = int(camera.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
    center_min, center_max = get_center()

    # setup the mouse callback
    cv2.namedWindow("frame")
    cv2.setMouseCallback("frame", selectROI)

    # initialize the termination criteria for cam shift, indicating
    # a maximum of ten iterations or movement by a least one pixel
    # along with the bounding box of the ROI
    termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
    roiBox = None

    # keep looping over the frames
    while True:
        # grab the current frame
        (grabbed, frame) = camera.read()

        # check to see if we have reached the end of the
        # video
        if not grabbed:
            break

        # if the see if the ROI has been computed
        if roiBox is not None:
            # convert the current frame to the HSV color space
            # and perform mean shift
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)

            # apply cam shift to the back projection, convert the
            # points to a bounding box, and then draw them
            (r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
            pts = np.int0(cv2.cv.BoxPoints(r))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            # discover robot positon and calculate next move
            xPosition = r[0][0]
            direction = get_direction(xPosition)
            area = cv2.contourArea(pts)

            if (area > default_area and direction == 'center'):
                #backward
                publish_movement('<')
            elif (area < default_area and direction == 'center'):
                # forward
                publish_movement('i')
            else:
                #left right
                if (direction == 'right'):
                    publish_movement('o')
                else:
                    publish_movement('u')


        # show the frame and record if the user presses a key
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # handle if the 'i' key is pressed, then go into ROI
        # selection mode
        if key == ord("i") and len(roiPts) < 4:
            # indicate that we are in input mode and clone the
            # frame
            inputMode = True
            orig = frame.copy()

            # keep looping until 4 reference ROI points have
            # been selected; press any key to exit ROI selction
            # mode once 4 points have been selected
            while len(roiPts) < 4:
                cv2.imshow("frame", frame)
                cv2.waitKey(0)

            # determine the top-left and bottom-right points
            roiPts = np.array(roiPts)
            s = roiPts.sum(axis = 1)
            tl = roiPts[np.argmin(s)]
            br = roiPts[np.argmax(s)]

            # grab the ROI for the bounding box and convert it
            # to the HSV color space
            roi = orig[tl[1]:br[1], tl[0]:br[0]]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

            # compute a HSV histogram for the ROI and store the
            # bounding box
            roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
            roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
            roiBox = (tl[0], tl[1], br[0], br[1])

        # if the 'q' key is pressed, stop the loop
        elif key == ord("q"):
            break

    # cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()


    def publish_movement(key_binding):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        speed = rospy.get_param("~speed", 0.5)
    	turn = rospy.get_param("~turn", 1.0)
    	x = 0
    	y = 0
    	z = 0
    	th = 0
    	status = 0
    	try:
    		print msg
    		print vels(speed,turn)
    		while(1):
    			key = key_binding
    			if key in moveBindings.keys():
    				x = moveBindings[key][0]
    				y = moveBindings[key][1]
    				z = moveBindings[key][2]
    				th = moveBindings[key][3]
    			# elif key in speedBindings.keys():
    			# 	speed = speed * speedBindings[key][0]
    			# 	turn = turn * speedBindings[key][1]
    			# 	print vels(speed,turn)
    			# 	if (status == 14):
    			# 		print msg
    			# 	status = (status + 1) % 15
    			# else:
    			# 	x = 0
    			# 	y = 0
    			# 	z = 0
    			# 	th = 0
    			# 	if (key == '\x03'):
    			# 		break
    			twist = Twist()
    			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
    			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    			pub.publish(twist)
    	except:
    		print e
    	finally:
    		twist = Twist()
    		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    		pub.publish(twist)
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }
speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

if __name__ == "__main__":
    main()
