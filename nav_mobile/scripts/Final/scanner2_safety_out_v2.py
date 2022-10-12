#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from lust_msgs.msg import multi_bool


class SICK_safety_out():
        
    def __init__(self):
        # init variables
        self.intrusion_msg = multi_bool()
        # set loop frequency to 10 Hz
        self.rate = rospy.Rate(25)

        
        # define subscriber
        self.sick_sub = rospy.Subscriber("/scanner2/sick_safetyscanners/scan", LaserScan, self.sick_callback)
        # define publisher
        self.safe_out_pub = rospy.Publisher("/scanner2_data", multi_bool, queue_size=10)


        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c

        self.ctrl_c = True
        
    def sick_callback(self, data):
        # check  3 fields
        self.intrusion_msg = multi_bool()

        for ii in range(1047):
            laser_measurement = data.ranges[ii]
            
            # field one
            if (laser_measurement) < 0.5:
                self.intrusion_msg.back_danger_thin = True
                # print('zona 1')
                # return

            # field two
            if (laser_measurement) < 1:
                self.intrusion_msg.back_danger_wide = True
                # print('zona 2')
                
            # field three
            if (laser_measurement) < 2:
                self.intrusion_msg.back_alert = True
                # print('zona 3')
    
        
        self.safe_out_pub.publish(self.intrusion_msg)


    def publish_once(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.safe_out_pub.get_num_connections()
            if connections > 0:
                self.safe_out_pub.publish(self.intrusion_msg)
                #rospy.loginfo("Msg Published")
                break
            else:
                self.rate.sleep()

    def just_wait(self):

        rospy.spin()
    

if __name__ == '__main__':
    # initialise node
    rospy.init_node('sick_safety_fields_s2', anonymous=True)

    # initialise class
    sso = SICK_safety_out()
    try:
        sso.just_wait()
    except rospy.ROSInterruptException:
        pass
    
    

