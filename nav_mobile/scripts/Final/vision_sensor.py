#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from copy import deepcopy



class fake_vision():

    def __init__(self):

        # define msg
        self.msg = Bool()
        # define rate
        self.rate = rospy.Rate(10)
        
        self.old_data = Bool()
        self.old_data.data = False

        self.cnt = 0
        
        # TODO FIX NAMES
        self.vision_status_sub = rospy.Subscriber("/platformStatus", Bool, self.vision_callback)
        # TODO FIX COMMENTS
        self.vision_status_pub = rospy.Publisher('/visionStatus', Bool, queue_size=10)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):

        # reset all values        
        self.msg = False
        self.publish_once()

        self.ctrl_c = True


    def vision_callback(self,input):
        

        self.msg.data = False

        # print((input.data == True) and (self.old_data.data == False))

        if (input.data == True) and (self.old_data.data == False):
            print("spim")
            rospy.sleep(3)
            self.msg.data = True

        # print("Data " + str(input.data) + " :" + str((input.data == True)))
        # print("Old Data " + str(self.old_data.data) + " :" + str((self.old_data.data == False)))
        # print("Msg " + str(self.msg))

        self.old_data = input
        

        # publish cmd
        self.publish_once()    
            


    def publish_once(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vision_status_pub.get_num_connections()
            if connections > 0:
                self.vision_status_pub.publish(self.msg)
                rospy.loginfo("Msg Published" + str(self.msg))
                break
            else:
                self.rate.sleep()

    def just_wait(self):

        # just wait for new input
        #  while not self.ctrl_c:

        #     self.vision_status_pub.publish(self.msg)
        #     self.rate.sleep()

        rospy.spin()

        


if __name__ == '__main__':
    
    # initialise node
    rospy.init_node('fake_vision', anonymous=True)

    # initialise class
    fv = fake_vision()
    try:
        fv.just_wait()
    except rospy.ROSInterruptException:
        pass

