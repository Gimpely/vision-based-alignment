#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from copy import deepcopy



class joy2joy2():

    def __init__(self):

        # define msg
        self.msg = Joy()
        # define rate
        self.rate = rospy.Rate(10)
        # define default gains
        
        # define publisher to /cmd_vel
        self.joy_pub = rospy.Publisher('/joy2', Joy, queue_size=1)
        # define /joy subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):

        # reset all values        
        self.msg = Joy()
        self.msg.axes = [0, 0, 0, 0, 0, 0]
        self.msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.publish_once()

        self.ctrl_c = True

    def joy_callback(self,data):
        
        # GENERIC
        # axis: 
        # L/R         0
        # U/D         1
        # buttons:
        # L           4
        # R           5
        # Start       9
        # Select/Bck  8
        # Yellow      2
        # Green       3
        # Red         1
        # Blue        0

        self.msg = deepcopy(data)
        self.msg.axes = [data.axes[0], data.axes[1], 0, 0, 0, 0]
        btns = data.buttons
        self.msg.buttons = [btns[3], btns[2], btns[1], btns[0], btns[4], btns[5], 0, 0, btns[8], btns[9], 0, 0]
        
        # publish cmd
        self.publish_once()    
            


    def publish_once(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.joy_pub.get_num_connections()
            if connections > 0:
                self.joy_pub.publish(self.msg)
                #rospy.loginfo("Msg Published")
                break
            else:
                self.rate.sleep()

    def just_wait(self):

        # just wait for new input
        rospy.spin()

        


if __name__ == '__main__':
    
    # initialise node
    rospy.init_node('joy_to_joy2', anonymous=True)

    # initialise class
    jj2 = joy2joy2()
    try:
        jj2.just_wait()
    except rospy.ROSInterruptException:
        pass

