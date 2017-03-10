#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

rospy.init_node('bumper_stop')

class BumperStop(object):
    def __init__(self):
        super(BumperStop, self).__init__()
        self.cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bmpSub = rospy.Subscriber('/bump', Bump, self.bumper_callback)
        self.bumped = False

    def bumper_callback(self, msg):
        self.bumped = msg.leftFront or msg.leftSide or msg.rightFront or msg.rightSide

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            speed = 0 if self.bumped else 0.1
            velMsg = Twist(linear=Vector3(x = speed), angular=Vector3(z=0))
            self.cmdPub.publish(velMsg)
            r.sleep()



if __name__ == '__main__':
    BumperStop().run()
