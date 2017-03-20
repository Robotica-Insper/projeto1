#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
import smach
import smach_ros



rospy.init_node('bumper_stop')

class BumperStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = 'bumperON', 'bumperOFF')
        ### self.counter = 0 Pra que serve essa linha?
        super(BumperStop, self).__init__()
        self.cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bmpSub = rospy.Subscriber('/bump', Bump, self.bumper_callback)
        self.bumped = False

#Criar intermédiario 



    def bumper_callback(self, msg):
        self.bumped = msg.leftFront or msg.leftSide or msg.rightFront or msg.rightSide

    
    def run(smach.State):
        def __init__(self):
        smach.State.__init__(self, outcomes=['bumperOFF'])
        
        r = rospy.Rate(10000)
	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
        while not rospy.is_shutdown():
            if self.bumped:
                print ("bump")
                x = -1
                y = 0
                z = 0
                th = -1
                return 'bumperON'

            else:
                x = 1
                y = 0
                z = 0
                th = 0

                return 'bumperOFF'
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            self.cmdPub.publish(twist)
            r.sleep()


             def execute(self, userdata):
                
                global speed
                rospy.loginfo('Executing state bumperON')
                vel = Twist(Vector3(-0,5, 0, 0), Vector3(0, 0, 0,5))
                velocidade_saida.publish(vel)
                

                return run







            def main():
      
                rospy.init_node('smach_example_state_machine')
                velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

                 # Create a SMACH state machine
                sm = smach.StateMachine(outcomes=['terminei'])

                 # Open the container
                 with sm:
                 # Add states to the container
                 smach.StateMachine.add('bumperOFF', run(), 
                               transitions={'bumperOFF':'run', 
                                            'bumperON':'execute'})
                 smach.StateMachine.add('bumperOFF', run()) 
                               

                # Execute SMACH plan
                outcome = sm.execute()


if __name__ == '__main__':
    BumperStop().run()
