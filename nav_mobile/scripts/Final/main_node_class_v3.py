#!/usr/bin/env python
#from __future__ import print_function
from pickle import TRUE
#from re import I
import time as t
from turtle import color
import rospy
#from sick_safetyscanners.msg import RawMicroScanDataMsg as IDM
from geometry_msgs.msg import Vector3, Twist
from lust_msgs.msg import greenhouse_cmd, multi_bool
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from copy import deepcopy, copy
from colorama import Fore,Back, Style
import os


 

class greenbot_safe_control():
    def __init__(self):

        self.time0 = rospy.Time.now()
        self.time1 = rospy.Time.now()
        self.flag = 0
        self.flag2 = 0
        self.flag_old = 999
        self.safety_signal_old2 = Bool()
        self.safety_signal_old = Bool()

        # self._check_laser1_ready()
        self._check_laser2_ready()

        # if lasers are ok, then p
        rospy.loginfo("The Greenbot is READY!" + "\n")

        self.vel_msg = greenhouse_cmd()
        self.joy_vel = greenhouse_cmd()
        self.safety_areas = multi_bool()
        
        # self.pub = rospy.Publisher("/safe_vel", lust_cmd, queue_size=25)
        # time publisher
        self.time_pub = rospy.Publisher("/platform_time", Int32, queue_size=25)
        # publisher to greenbot
        self.safe_vel_pub = rospy.Publisher("/safe_vel", greenhouse_cmd, queue_size=25)
        # publisher to arduino - LED status
        self.led_pub = rospy.Publisher("/LED_status", Int32, queue_size=10)
        # subscriber to nanoscan - front
        self.safe_sub1 = rospy.Subscriber("/scanner1_data", multi_bool, self.scanner1_safe_callback)
        # substriber to nanoscan - back
        self.safe_sub2 = rospy.Subscriber("/scanner2_data", multi_bool, self.scanner2_safe_callback)
        # subscriber to /cmd_vel from joystic
        self.vel_sub = rospy.Subscriber("/cmd_vel", greenhouse_cmd, self.vel_callback)
        # subscriber to /odom from the robot
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
         # TODO FIX NAMES
        self.vision_status_pub = rospy.Publisher("/platformStatus", Bool, queue_size=10)
        # TODO FIX NAMES
        # self.vision_mode_pub = rospy.Publisher("/platformMode", Int32, queue_size=10)
        # TODO FIX COMMENTS
        self.vision_status_sub = rospy.Subscriber('/visionStatus', Bool, self.vision_callback)

        self.ctrl_c = False
        self.rate = rospy.Rate(10)

        self.fwd_first_odom_read = False
        self.fw_running = False
        self.fw_running_pause = False
        self.init_back = True
        self.init_back_start = 0

        # odometry
        self.odom = Odometry()
        self.odom_start = 0
        self.odom_middle = 0
        self.end_flag = False
        self.odometry_offset = 0
        self.odometry_offset_updated = False # reset - True, no reset - False

        # pausing movement parameters
        self.odom_pause = 0
        self.flag_pause = 0
        self.sensor_start = False
        self.sensor_done = True
        self.number_pause_measurement = 0
        self.bwd_first_odom_read = True


        self.platform_status = 6 #Idle BLUE
        self.platform_status_old = 0 #Idle BLUE

        rospy.on_shutdown(self.shutdownhook)

    def scanner1_safe_callback(self, data):
        self.safety_areas.front_danger_thin = data.front_danger_thin
        self.safety_areas.front_danger_right = data.front_danger_right
        self.safety_areas.front_danger_wide = data.front_danger_wide
        self.safety_areas.front_alert = data.front_alert
        self.safety_areas.front_alert_right = data.front_alert_right
    
    def scanner2_safe_callback(self, data):
        self.safety_areas.back_danger_thin = data.back_danger_thin
        self.safety_areas.back_danger_left = data.back_danger_left
        self.safety_areas.back_danger_wide = data.back_danger_wide
        self.safety_areas.back_alert = data.back_alert
        self.safety_areas.back_alert_left = data.back_alert_left

    def vel_callback(self, msg):
        self.joy_vel = deepcopy(msg)
        self.vel_msg = msg
        # multiply with velocity gain
        self.vel_msg.cmd_vel.linear.x = self.vel_msg.cmd_vel.linear.x * rospy.get_param("/velocity_gain")
        self.vel_msg.cmd_vel.angular.z = self.vel_msg.cmd_vel.angular.z * rospy.get_param("/velocity_gain")


    def odometry_callback(self, odo_send):
        self.odom = odo_send

    def _check_laser1_ready(self):
        self.laser_msg1 = None
        rospy.logdebug("Checking Sick NanoScan3 laser...")
        while self.laser_msg1 is None and not rospy.is_shutdown():
            try:
                self.laser_msg1 = rospy.wait_for_message("/scanner1_data", multi_bool, timeout=1.0)
                rospy.logdebug("Current /scanner1_data READY=>" + str(self.laser_msg1))
            except:
                rospy.logerr("Current /scanner1_data not ready yet, retrying for getting scan")
        rospy.loginfo("Checking SICK Nanoscan3 1...DONE")
        return self.laser_msg1

    def _check_laser2_ready(self):
        self.laser_msg2 = None
        rospy.logdebug("Checking Sick NanoScan3 laser...")
        while self.laser_msg2 is None and not rospy.is_shutdown():
            try:
                self.laser_msg2 = rospy.wait_for_message("/scanner2_data", multi_bool, timeout=1.0)
                rospy.logdebug("Current /scanner2_data READY=>" + str(self.laser_msg2))
            except:
                rospy.logerr("Current /scanner2_data not ready yet, retrying for getting scan")
        rospy.loginfo("Checking SICK Nanoscan3 2...DONE" + "\n")
        return self.laser_msg2

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_robot()
        self.ctrl_c = True
        


    ############ Stop functions ############################################################
    def stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        #self.pub3.publish(self.platform_status)
        self.platform_status = 0 #Stop RED
        self.fw_running = False
        self.safe_vel_pub.publish(self.vel_msg)    
        # self.pub.publish(self.vel_msg)
        if self.vel_msg.mode == 0:
            self.vision_status_pub.publish(True) 

    def manual_stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        self.platform_status = 0 #Stop RED 
        if self.platform_status != self.platform_status_old:
            rospy.loginfo("STOP button pressed.")
            self.platform_status_old = self.platform_status

        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)
        if self.vel_msg.mode == 0:
            self.vision_status_pub.publish(True)   

    def auto_stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        #self.pub3.publish(self.platform_status)
        self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        if self.platform_status != self.platform_status_old:
            rospy.logdebug("AUTO STOP of the greenbot. Obstacle detected.")
            self.platform_status_old = self.platform_status
        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)
        if self.vel_msg.mode == 0:
            self.vision_status_pub.publish(True)     

    def auto_stop_pause(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        #self.pub3.publish(self.platform_status)
        self.platform_status = 7 #Obstacle AUTO Blinking YELLOW - takon picture
        rospy.logdebug("AUTO STOP of the greenbot. Taking picture.")
        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)
        # Tell vision that platform is not moveing
        self.vision_status_pub.publish(True)  



    ############ Slowdown and idle functions ############################################################
    def slowdown_alert(self):
        self.vel_msg.cmd_vel.linear.x = self.joy_vel.cmd_vel.linear.x*0.5
        # self.vel_msg.cmd_vel.angular.x = self.joy_vel.cmd_vel.angular.x*0.5
        self.vel_msg.cmd_vel.angular.z = self.joy_vel.cmd_vel.angular.z*0.5
        #self.pub3.publish(self.platform_status)
        
        self.platform_status = 1 #Alert signal YELLOW 
        if self.platform_status != self.platform_status_old:
           rospy.logdebug("Slowdown enabled.")
           self.platform_status_old = self.platform_status       
        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)   

    def idle(self):
        self.vel_msg.cmd_vel.linear.x = 0
        self.vel_msg.cmd_vel.angular.x = 0
        self.vel_msg.cmd_vel.angular.z = 0
        #self.pub3.publish(self.platform_status)
        
        self.platform_status = 6 #Idle BLUE 
        if self.platform_status != self.platform_status_old:
           rospy.logdebug("IDLE.")
           self.platform_status_old = self.platform_status       
        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)   

    ############ Varnost ob premikih naravnost naprej ############################################################
    def forward_movement(self):
        #print("forward")
        if (self.safety_areas.front_danger_right == True) or (self.safety_areas.back_danger_left == True) or (self.safety_areas.front_danger_wide == True) or (self.vel_msg.stop_run == True):
            rospy.loginfo("[Sensor 1]:  Danger close! Stopping the greenbot.]")
            self.stop_robot()
        elif ((self.safety_areas.front_danger_wide == True) and (self.safety_areas.front_alert == True)) or (self.vel_msg.stop_run == True):
            rospy.loginfo("[Sensor 1]:  Danger close! Stopping the greenbot.]")
            self.stop_robot()    
        elif self.safety_areas.front_alert == True:
            self.slowdown_alert()
        else:
            self.platform_status = 4 #Joystick control GREEN
            # self.pub.publish(self.vel_msg) 
            self.safe_vel_pub.publish(self.vel_msg)   

    ############  Varnost ob premikih naravnost nazaj ############################################################
    def backward_movement(self):
        #print("backward")
        if (self.safety_areas.front_danger_right == True) or (self.safety_areas.back_danger_left == True) or (self.safety_areas.back_danger_wide == True) or (self.vel_msg.stop_run == True):
            rospy.loginfo("[Sensor 2]:  Danger close! Stopping the greenbot.]")
            self.stop_robot()
        elif ((self.safety_areas.back_danger_wide == True) and (self.safety_areas.back_alert == True)) or (self.vel_msg.stop_run == True):
            rospy.loginfo("[Sensor 2]:  Danger close! Stopping the greenbot.]")
            self.stop_robot() 
        elif self.safety_areas.back_alert == True:
            self.slowdown_alert()
        else:
            self.platform_status = 4 #Joystick control GREEN
            # self.pub.publish(self.vel_msg) 
            self.safe_vel_pub.publish(self.vel_msg)    

    ############ Varnost ob zavijanju in vrtenju ############################################################
    def rotational_movement(self):
        #print("rotational")
        if (self.safety_areas.back_danger_wide == True) or (self.safety_areas.front_danger_wide == True) or (self.safety_areas.front_danger_right == True) or (self.safety_areas.back_danger_left == True) or (self.vel_msg.stop_run == True):
            rospy.loginfo("[Sensor 1 & 2 (side)]:  Danger close! Stopping the greenbot.]")
            self.stop_robot()
        else:    
            if (self.safety_areas.back_alert == True) or (self.safety_areas.back_alert_left == True) or (self.safety_areas.front_alert == True) or (self.safety_areas.front_alert_right == True):
                self.slowdown_alert()
            else:
                self.platform_status = 4 #Joystick control GREEN
                # self.pub.publish(self.vel_msg) 
                self.safe_vel_pub.publish(self.vel_msg)      

    ############ Pocasnejsa voznja naravnost in nazaj (upostevanje senzorjev) ####################################
    def slower_auto_movement_forward(self):
        if self.safety_areas.front_danger_thin == False:
            self.vel_msg.cmd_vel.linear.x = 0.75 * rospy.get_param("/velocity_gain")
            # self.vel_msg.cmd_vel.angular.x = 0
            self.vel_msg.cmd_vel.angular.z = 0
            self.platform_status = 2 #Auto forward VIOLET
            #self.pub3.publish(self.platform_status)
            # self.pub.publish(self.vel_msg)
            self.safe_vel_pub.publish(self.vel_msg)
            # Tell vision that platform is moveing
            self.vision_status_pub.publish(False)   
        elif self.safety_areas.front_danger_thin == True or self.vel_msg.stop_run == True:
            rospy.loginfo("[Sensor 1]:  Danger close! Stopping the greenbot.]")
            self.flag = 999
            self.stop_robot()

    def slower_auto_movement_backward(self):
        if self.safety_areas.back_danger_thin == False:
            self.vel_msg.cmd_vel.linear.x = -0.75 * rospy.get_param("/velocity_gain")
            # self.vel_msg.cmd_vel.angular.x = 0
            self.vel_msg.cmd_vel.angular.z = 0.0
            self.platform_status = 2 #Auto forward VIOLET
            # self.pub.publish(self.vel_msg)
            self.safe_vel_pub.publish(self.vel_msg)
            # Tell vision that platform is moveing
            self.vision_status_pub.publish(False)   
        elif self.safety_areas.back_danger_thin == True or self.vel_msg.stop_run == True:
            rospy.loginfo("[Sensor 2]:  Danger close! Stopping the greenbot.]")
            self.flag = 999
            self.stop_robot()

    ############ Override safety movement (no sensor data used) #####################################################
    def manual_override_safety_movement(self):
        
        self.vel_msg.cmd_vel.linear.x = self.joy_vel.cmd_vel.linear.x*0.5
        # self.vel_msg.cmd_vel.angular.x = self.joy_vel.cmd_vel.angular.x*0.5
        self.vel_msg.cmd_vel.angular.z = self.joy_vel.cmd_vel.angular.z*0.5
        #self.pub3.publish(self.platform_status)
        self.platform_status = 5 #Override joystick control BLINKING RED 
        if self.platform_status != self.platform_status_old:
           rospy.loginfo("[ Override safety!!! ]")
           self.platform_status_old = self.platform_status        
        # self.pub.publish(self.vel_msg)
        self.safe_vel_pub.publish(self.vel_msg)   


    ############ Avtonomni premiki ########################################################################
    def safety_auto_movement(self):
        if self.fwd_first_odom_read:
            self.odom_start = self.odom.pose.pose.position.x
            self.fwd_first_odom_read = False
            self.fw_running = True
        self.flag = 0
        if self.safety_areas.front_danger_thin == True:
            self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
            self.flag = 1   
            if not self.safety_signal_old.data:
                self.time0 = rospy.Time.now()
            
            dt = rospy.Time.now()
            dt = dt - self.time0

            if dt.to_sec() > 5:
                self.flag = 2

        self.odom_middle = self.odom.pose.pose.position.x        
        rospy.logdebug("Odometrija ob pritisku tipke: " +  str(self.odom_start))
        rospy.logdebug("Odometrija ob vracanju nazaj: " + str(self.odom_middle))
        if self.flag_old == 2 or self.flag_old == 3:
            self.flag = 2
            if ((self.odom_middle - self.odom_start) <= 0):
                self.flag = 999 
                self.fw_running = False
                self.end_flag = True
            elif (self.safety_areas.back_danger_thin == True):
                self.flag = 999
                self.flag = 3 
                self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        else:
            self.end_flag = True
            self.flag = self.flag

        self.safety_signal_old.data = self.safety_areas.front_danger_thin
        self.flag_old = self.flag



    def safety_auto_movement_fwd(self):

        # reset odometry
        if self.fwd_first_odom_read:
            current_pose = self.odom.pose.pose.position.x - self.odometry_offset
            self.odom_start = current_pose
            self.fwd_first_odom_read = False
            self.fw_running = True
            # print("Goal pose " + str(self.odom_start))
        
        # enable auto movement - flag = 0
        self.flag = 0
        
        # check for obstacles
        if self.safety_areas.front_danger_thin == True:
            self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
            self.flag = 1   

            # if this is new obstacle, start timer
            if not self.safety_signal_old.data:
                self.time0 = rospy.Time.now()
            
            dt = rospy.Time.now()
            dt = dt - self.time0

            # if difference is larger then 10 s, send a report and stop the greenbot
            if dt.to_sec() > rospy.get_param("/end_waiting_time"):
                rospy.loginfo("Obstacle detected for more than " + str(rospy.get_param("/end_waiting_time")) + " s. Please check or return greenbot.")
                rospy.logdebug("Greenbot traveled " + str(self.odom.pose.pose.position.x - self.odometry_offset) + " mm.")
                self.fw_running = False
                self.flag = 999
                return

        self.safety_signal_old.data = self.safety_areas.front_danger_thin


    def safety_auto_movement_bwd(self):

        # check if odometry reset is updated
        current_pose = self.odom.pose.pose.position.x - self.odometry_offset

        rospy.logdebug("Current pose " + str(current_pose))
        rospy.logdebug("Goal pose " + str(self.odom_start))
                
        # enable auto movement - flag = 0
        self.flag = 0

        # determine direction
        if self.init_back:
            self.init_back_start = current_pose
            self.init_back = False

        if self.init_back_start > self.odom_start:      
            if (current_pose - self.odom_start) <= 0:
                # ce je prisel robot do konca, se ustavi
                rospy.loginfo("Robot returned. Current pose: " + str(current_pose) + " mm.")
                self.flag = 999
                self.init_back = True
                return 
            elif (self.safety_areas.back_danger_thin == True):
                # if obstacle detected, stop
                self.flag = 1 
                self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        else:
            if (current_pose - self.odom_start) >= 0:
                # ce je prisel robot do konca, se ustavi
                rospy.loginfo("Robot returned. Current pose: " + str(current_pose) + " mm.")
                self.flag = 999
                self.init_back = True
                return 
            elif (self.safety_areas.back_danger_thin == True):
                # if obstacle detected, stop
                self.flag = 1 
                self.platform_status = 3 #Obstacle AUTO Blinking YELLOW


    
    
    ############ Avtonomni premiki s pavzami ############################################################
                        
    def safety_pausing_auto_movement_fwd(self):
        # update platform status
        self.platform_status = 300
        # read the odometry
        if self.fwd_first_odom_read:
            self.odom_start = self.odom.pose.pose.position.x - self.odometry_offset
            self.odom_pause = self.odom.pose.pose.position.x - self.odometry_offset
            self.fwd_first_odom_read = False
            self.fw_running = True
            self.number_pause_measurement = 0
            self.sensor_done = True
            
        
        # preveri, ce si dobil info iz senzorja, da je koncal
        if self.sensor_done:

            self.flag_pause = 0
            self.sensor_start = False
            # self.publish_once_platform_status(self.sensor_start) 

            current_pose = self.odom.pose.pose.position.x - self.odometry_offset

            # check if the distance achieved 
            if (current_pose - self.odom_pause) >= rospy.get_param("/stop_distance"):
                rospy.loginfo("Stop distance traveled (" + str(current_pose) + " mm, number of measurements: " + str(self.number_pause_measurement+1) +"), start scanning.")
                self.flag_pause = 1
                # ponastavi zadnjo znano pozicijo
                self.odom_pause = current_pose
                # poslji ustrezen signal, da se je platforma ustavila
                self.sensor_start = True
                # sensor_done mora iti na False!
                self.sensor_done = False
                # povecaj st meritev
                self.number_pause_measurement += 1
                #self.vision_status_pub.publish(self.sensor_start)
                #self.publish_once_platform_status(self.sensor_start) 
                # self.vision_status_pub.publish(self.sensor_start) 
                return

            # check for obstacles
            if self.safety_areas.front_danger_thin == True:
                self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
                self.flag_pause = 2  

                # if this is new obstacle, start timer
                if not self.safety_signal_old.data:
                    self.time0 = rospy.Time.now()
                
                dt = rospy.Time.now()
                dt = dt - self.time0

                # if difference is larger then 10 s, send a report and stop the greenbot
                if dt.to_sec() > rospy.get_param("/end_waiting_time"):
                    rospy.loginfo("Obstacle detected for more than " + str(rospy.get_param("/end_waiting_time")) + " s. Please check or return greenbot.")
                    rospy.logdebug("Greenbot traveled " + str(self.odom.pose.pose.position.x - self.odometry_offset) + " mm.")
                    self.flag_pause = 999
                    self.fw_running = False
                    return
        else:
            # pocakaj, dokler senzor ni pripravljen
            self.flag_pause = 1

        self.safety_signal_old.data = self.safety_areas.front_danger_thin


    def safety_pausing_auto_movement_bwd(self):
        # update platform status
        self.platform_status = 300
        # read the odometry
        if self.bwd_first_odom_read:
            self.odom_pause = self.odom.pose.pose.position.x - self.odometry_offset
            self.bwd_first_odom_read = False
            self.number_pause_measurement = 0
            self.init_back_start = self.odom.pose.pose.position.x - self.odometry_offset
            self.sensor_done = True
            
        
        # preveri, ce si dobil info iz senzorja, da je koncal
        if self.sensor_done:

            self.flag_pause = 0
            self.sensor_start = False
            # self.publish_once_platform_status(self.sensor_start) 

            current_pose = self.odom.pose.pose.position.x - self.odometry_offset

            # check if the distance achieved 
            if (self.odom_pause - current_pose) >= rospy.get_param("/stop_distance"):
                rospy.loginfo("Stop distance traveled (" + str(current_pose) + " mm, number of measurements: " + str(self.number_pause_measurement) +"), start scanning.")
                self.flag_pause = 1
                # ponastavi zadnjo znano pozicijo
                self.odom_pause = current_pose
                # poslji ustrezen signal, da se je platforma ustavila
                self.sensor_start = True
                # sensor_done mora iti na False!
                self.sensor_done = False
                # povecaj st meritev
                self.number_pause_measurement += 1
                #self.vision_status_pub.publish(self.sensor_start)
                #self.publish_once_platform_status(self.sensor_start) 
                return

         
            if self.init_back_start > self.odom_start:      
                if (current_pose - self.odom_start) <= 0:
                    # ce je prisel robot do konca, se ustavi
                    rospy.loginfo("Robot returned. Current pose: " + str(current_pose) + " mm.")
                    self.flag_pause = 999
                    self.bwd_first_odom_read = True
                    return 
                elif (self.safety_areas.back_danger_thin == True):
                    # if obstacle detected, stop
                    self.flag = 1 
                    self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
            else:
                if (current_pose - self.odom_start) >= 0:
                    # ce je prisel robot do konca, se ustavi
                    rospy.loginfo("Robot returned. Current pose: " + str(current_pose) + " mm.")
                    self.flag_pause = 999
                    self.bwd_first_odom_read = True
                    return 
                elif (self.safety_areas.back_danger_thin == True):
                    # if obstacle detected, stop
                    self.flag = 1 
                    self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        else:
            # pocakaj, dokler senzor ni pripravljen
            self.flag_pause = 2


    ############ Premiki s kontrolerjem (upostevanje stop tipke) ############################################################
    def manual_movement(self):
        if self.vel_msg.stop_run == True:
            self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
            self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
            self.platform_status = 0 #Stop RED 
            # self.pub.publish(self.vel_msg)
            self.safe_vel_pub.publish(self.vel_msg)   
        else:
            self.platform_status = 4 #Joystick control GREEN
            #self.pub3.publish(self.platform_status)
            # self.pub.publish(self.vel_msg)
            self.safe_vel_pub.publish(self.vel_msg) 


    ############ VISION funkcije ####################################################################################
    
    def vision_callback(self, data):

        # data je tip Bool(), bool ima polje data
        self.sensor_done = data.data

        
    def publish_once_platform_status(self, data):
            """
            This is because publishing in topics sometimes fails the first time you publish.
            In continuous publishing systems, this is no big deal, but in systems that publish only
            once, it IS very important.
            """
            sensor_msg = Bool()
            sensor_msg.data = data

            while not self.ctrl_c:
                connections = self.platform_status_pub.get_num_connections()
                print("Bom poslal. Povezav je " + str(connections))
                if connections > 0:
                    self.platform_status_pub.publish(sensor_msg)
                    rospy.loginfo("Msg Published")
                    break
                else:
                    self.rate.sleep()


    ############ MAIN function ####################################################################################
    def ctrl_to_communication(self):
            while not self.ctrl_c:
                if not self.fw_running:
                    self.fwd_first_odom_read = True
        
                # print("Odoma puse: " + str(self.odom_pause))
                # print("Odom start: " + str(self.odom_start))
                # avtomatska voznja naprej
                if (self.vel_msg.start_run_fwd == True) and (self.vel_msg.mode == 0) and (self.flag == 999):
                    self.stop_robot()
                elif (self.vel_msg.start_run_fwd == True) and (self.vel_msg.mode == 0) and (self.flag != 999):
                    # self.safety_auto_movement()
                    self.safety_auto_movement_fwd()
                    if self.flag == 0:
                        self.slower_auto_movement_forward()
                    if self.flag == 1:
                        self.auto_stop_robot()
                # avtomatska voznja nazaj na podlagi odometrije
                elif (self.vel_msg.start_run_bwd == True) and (self.vel_msg.mode == 0) and (self.flag == 999):
                    self.stop_robot()
                elif (self.vel_msg.start_run_bwd == True) and (self.vel_msg.mode == 0) and (self.flag != 999):
                    # self.safety_auto_movement()
                    self.safety_auto_movement_bwd()
                    if self.flag == 0:
                        self.slower_auto_movement_backward()
                    if self.flag == 1:
                        self.auto_stop_robot()
            
                
                # start pausing run forward
                elif (self.vel_msg.start_run_fwd == True) and (self.vel_msg.mode == 1) and (self.flag_pause == 999):
                    self.stop_robot()
                elif (self.vel_msg.start_run_fwd == True) and (self.vel_msg.mode == 1) and (self.flag_pause != 999):
                    self.safety_pausing_auto_movement_fwd()
                    if self.flag_pause == 0:
                        self.slower_auto_movement_forward()
                    if self.flag_pause == 1:
                        self.auto_stop_pause()
                    if self.flag_pause == 2:
                        self.auto_stop_robot()
                # start pausing run backward
                elif (self.vel_msg.start_run_bwd == True) and (self.vel_msg.mode == 1) and (self.flag_pause == 999):
                    self.stop_robot()
                elif (self.vel_msg.start_run_bwd == True) and (self.vel_msg.mode == 1) and (self.flag_pause != 999):
                    self.safety_pausing_auto_movement_bwd()
                    if self.flag_pause == 0:
                        self.slower_auto_movement_backward()
                    if self.flag_pause == 1:
                        self.auto_stop_pause()
                    if self.flag_pause == 2:
                        self.auto_stop_robot()

                
                # manually stop robot
                elif (self.vel_msg.stop_run == True):
                    self.flag = 0
                    self.flag_pause = 0
                    # self.odom_start = self.odom.pose.pose.position.x
                    self.manual_stop_robot()


                # reset odometry
                elif self.vel_msg.reset_odometry == True:
                    self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
                    self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
                    # self.pub.publish(self.vel_msg)
                    self.safe_vel_pub.publish(self.vel_msg) 
                    
                    # set current odometry offset
                    self.odometry_offset = self.odom.pose.pose.position.x
                    rospy.loginfo("RESET odometry.")

                
                elif self.vel_msg.override_safety == True and self.vel_msg.stop_run == False:
                    self.manual_override_safety_movement()



                #premiki platforme naravnost (sprednji senzor ima glavno vlogo)
                elif (self.vel_msg.cmd_vel.linear.x > 0) and (self.vel_msg.cmd_vel.angular.z == 0) or ((self.vel_msg.cmd_vel.linear.x > 0) and (self.vel_msg.cmd_vel.angular.z == -0)):
                    self.forward_movement()     

                #zavijanje in vrtenje platforme na mestu (oba senzorja pomembna)
                elif (self.vel_msg.cmd_vel.angular.z != 0) and (self.vel_msg.cmd_vel.linear.x != 0):
                    self.rotational_movement()

                #zavijanje in vrtenje platforme na mestu (oba senzorja pomembna)
                elif ((self.vel_msg.cmd_vel.angular.z != 0) and (self.vel_msg.cmd_vel.linear.x == 0)):
                    self.rotational_movement()        

                #premiki platforme naravnost (zadnji senzor ima glavno vlogo)
                elif ((self.vel_msg.cmd_vel.linear.x < 0) and (self.vel_msg.cmd_vel.angular.z == 0)) or ((self.vel_msg.cmd_vel.linear.x < 0) and (self.vel_msg.cmd_vel.angular.z == -0)):
                    self.backward_movement()         
                
                else:
                    if self.vel_msg.stop_run == True:
                        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
                        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
                        self.platform_status = 0 #Stop RED 
                        # self.pub.publish(self.vel_msg)
                        self.safe_vel_pub.publish(self.vel_msg) 
                    else:
                        self.idle()
                
                self.led_pub.publish(self.vel_msg.mode)
                self.time_pub.publish(t.time())
                # self.vision_mode_pub.publish()
                self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node("greenbot_sending", anonymous=True, log_level=rospy.DEBUG)

    greenbot_control = greenbot_safe_control()
    try:
        greenbot_control.ctrl_to_communication()
    except rospy.ROSInterruptException:
        pass

