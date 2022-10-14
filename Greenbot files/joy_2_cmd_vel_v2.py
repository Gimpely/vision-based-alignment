#! /usr/bin/env python
from logging.config import stopListening
import rospy
from sensor_msgs.msg import Joy
from lust_msgs.msg import greenhouse_cmd


class joy2cmd():

    def __init__(self):

        # define msg
        self.msg = greenhouse_cmd()
        # define rate
        self.rate = rospy.Rate(10)
        # define default gains
        self.default_velocity_gain = rospy.get_param("/velocity_gain")
        self.default_stop_distance = rospy.get_param("/stop_distance") # in mm
        # self.msg.velocity_gain = self.default_velocity_gain
        # self.msg.stop_distance = self.default_stop_distance
        # define insane mode
        # self.insane_velocity_gain = 3

        # toggle btn
        self.btn_ = 0
        
        # define publisher to /cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', greenhouse_cmd, queue_size=1)
        # define /joy subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):

        # reset all values        
        self.msg = greenhouse_cmd()
        rospy.set_param("/velocity_gain", self.default_velocity_gain)
        rospy.set_param("/stop_distance",self.default_stop_distance)
        self.publish_once()

        self.ctrl_c = True

    def joy_callback(self,data):
        # A - automatic mode
        # X - automatic with stopListening
        # B - stop 
        # Y - reset odometry
        # START - run fwd
        # BACK - run back
        # LB - enable
        # RB + LB - override safety
        # left joystick - set velocity
        # cross up/down - set velocity gain
        # cross left/right - set stop distance
        # RT - reset parameters (gain, distance)
        # LR - insane mode (velocity_gain = 3)

        # manual drive mode
        if data.buttons[4] == 1:
            # set linear velocity
            self.msg.cmd_vel.linear.x = data.axes[1]
            # set rotational velocity
            self.msg.cmd_vel.angular.z = data.axes[0]

            # override safety
            if data.buttons[5] == 1:
                self.msg.override_safety = True
            else:
                self.msg.override_safety = False
        else:
            # set linear velocity
            self.msg.cmd_vel.linear.x = 0
            # set rotational velocity
            self.msg.cmd_vel.angular.z = 0

            # override safety    
            self.msg.override_safety = False

        # mode selection
        if data.buttons[0] == 1:
            # automatic with stops - CODE 1
            self.msg.mode = 1
        elif data.buttons[1] == 1:
            # automatic mode - CODE 0
            self.msg.mode = 0

        # reset odometry
        if data.buttons[3] == 1:
            self.msg.reset_odometry = True
        else:
            self.msg.reset_odometry = False

        # set parameters
        velocity_gain = rospy.get_param("/velocity_gain")
        stop_distance = rospy.get_param("/stop_distance")
        # velocity gain
        if data.axes[5] != 0:
            # velocity gain, step 10 %
            velocity_gain = rospy.get_param("/velocity_gain") + data.axes[5]*0.1
            # set limits
            if velocity_gain < 0.1:
                velocity_gain = 0.1
            elif velocity_gain > 1.5:
                velocity_gain = 1.5
            # self.msg.velocity_gain = velocity_gain
            rospy.set_param("/velocity_gain",velocity_gain)
            rospy.loginfo("Velocity gain set to " + str(velocity_gain))
        # stop distance
        if data.axes[4] != 0:
            # stop distance, step 0.1 m
            stop_distance = rospy.get_param("/stop_distance")- data.axes[4]*100
            if stop_distance > 5000:
                # max limit 5 m
                stop_distance = 5000
            elif stop_distance < 100:
                # min limit 0.1 m
                stop_distance = 100
            # self.msg.stop_distance = stop_distance
            rospy.set_param("/stop_distance", stop_distance)
            rospy.loginfo("Stop distance set to " + str(stop_distance))
        
        # insane mode!!!
        if data.buttons[6] == 1:
            if self.btn_ == 0:
                self.tmp_vel_gain = velocity_gain
            # self.msg.velocity_gain = self.insane_velocity_gain 
            insane_vel_gain = rospy.get_param("/insane_velocity_gain")
            rospy.set_param("/velocity_gain",insane_vel_gain)
            rospy.loginfo("[INSANE MODE] Velocity gain set to " + str(insane_vel_gain))
            self.btn_ = 1
        else:
            if self.btn_ == 1:
                # self.msg.velocity_gain = self.tmp_vel_gain
                rospy.set_param("/velocity_gain",self.tmp_vel_gain)
                rospy.loginfo("Velocity gain set to " + str(self.tmp_vel_gain))
            self.btn_ = 0

        
        # reset parameters to default
        if data.buttons[7] == 1:
            # self.msg.velocity_gain = self.default_velocity_gain
            rospy.set_param("/velocity_gain",self.default_velocity_gain)
            rospy.loginfo("Velocity gain reset to " + str(self.default_velocity_gain))
            # self.msg.stop_distance = self.default_stop_distance
            rospy.set_param("/stop_distance", self.default_stop_distance)
            rospy.loginfo("Stop distance reset to " + str(self.default_stop_distance))


        # automatic drive mode
        if data.buttons[9] == 1:
            # run forward
            self.msg.start_run_fwd = True
            self.msg.start_run_bwd = False
            self.msg.stop_run = False
        elif data.buttons[8] == 1:
            # run backward
            self.msg.start_run_bwd = True
            self.msg.start_run_fwd = False
            self.msg.stop_run = False
        elif data.buttons[2] == 1:
            # reset all
            tmp_mode = self.msg.mode
            self.msg = greenhouse_cmd()
            self.msg.mode = tmp_mode
            # self.msg.velocity_gain = velocity_gain
            # self.msg.stop_distance = stop_distance
            self.msg.stop_run = True
        elif data.buttons[2] == 0:
            # reset all
            self.msg.stop_run = False

        
        # publish cmd
        self.publish_once()    
            


    def publish_once(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.cmd_pub.get_num_connections()
            if connections > 0:
                self.cmd_pub.publish(self.msg)
                #rospy.loginfo("Msg Published")
                break
            else:
                self.rate.sleep()

    def just_wait(self):

        # just wait for new input
        rospy.spin()

        


if __name__ == '__main__':
    
    # initialise node
    rospy.init_node('joy_to_cmd', anonymous=True)

    # initialise class
    cmd = joy2cmd()
    try:
        cmd.just_wait()
    except rospy.ROSInterruptException:
        pass

