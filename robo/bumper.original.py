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
        r = rospy.Rate(100)
	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
        while not rospy.is_shutdown():
            if self.bumped:
                print ("bump")
                x = -1
                y = 0
                z = 0
                th = -1
            else:
                x = 1
                y = 0
                z = 0
                th = 0
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            self.cmdPub.publish(twist)
            r.sleep()



if __name__ == '__main__':
    BumperStop().run()
