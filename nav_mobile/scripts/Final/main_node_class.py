#!/usr/bin/env python
from __future__ import print_function
from ast import For
from pickle import TRUE
from re import I
from time import time
from turtle import color
import rospy
from sick_safetyscanners.msg import RawMicroScanDataMsg as IDM
from geometry_msgs.msg import Vector3, Twist
from lust_msgs.msg import lust_cmd, multi_bool
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from copy import deepcopy, copy
from colorama import Fore,Back, Style
import os


 

class Lust_safe_control():
    def __init__(self):

        rospy.init_node("lust_sending", anonymous=True)

        self.time0 = rospy.Time.now()
        self.time1 = rospy.Time.now()
        self.flag = 0
        self.flag2 = 0
        self.flag_old = 999  
        self.safety_signal_old2 = Bool()
        self.safety_signal_old = Bool()

        self._check_laser1_ready()
        self._check_laser2_ready()

        self.vel_msg = lust_cmd()
        self.joy_vel = lust_cmd()
        self.vel_msg.start_run = False
        self.vel_msg.stop_run = False
        self.vel_msg.override_safety = False
        self.vel_msg.reset_odometry = False
        self.vel_msg.cmd_vel.linear.x = 0
        self.vel_msg.cmd_vel.linear.y = 0
        self.vel_msg.cmd_vel.linear.z = 0
        self.vel_msg.cmd_vel.angular.x = 0
        self.vel_msg.cmd_vel.angular.y = 0
        self.vel_msg.cmd_vel.angular.z = 0
        self.joy_vel.cmd_vel.linear.x = 0
        self.joy_vel.cmd_vel.linear.y = 0
        self.joy_vel.cmd_vel.linear.z = 0
        self.joy_vel.cmd_vel.angular.x = 0
        self.joy_vel.cmd_vel.angular.y = 0
        self.joy_vel.cmd_vel.angular.z = 0
        self.front_danger_thin = multi_bool.front_danger_thin
        self.front_danger_right = multi_bool.front_danger_right
        self.front_danger_wide = multi_bool.front_danger_wide
        self.front_alert = multi_bool.front_alert
        self.front_alert_right = multi_bool.front_alert_right
        self.back_danger_thin = multi_bool.back_danger_thin
        self.back_danger_left = multi_bool.back_danger_left
        self.back_danger_wide = multi_bool.back_danger_wide
        self.back_alert = multi_bool.back_alert
        self.back_alert_left = multi_bool.back_alert_left
        self.odom_start_pause = 0.0

        self.pub = rospy.Publisher("/safe_vel", lust_cmd, queue_size=25)
        #self.pub2 = rospy.Publisher("/flagging", Bool, queue_size=10)
        self.pub3 = rospy.Publisher("/LED_status", Int32, queue_size=10)
        self.safe_sub1 = rospy.Subscriber("/scanner1_data", multi_bool, self.scanner1_safe_callback)
        self.safe_sub2 = rospy.Subscriber("/scanner2_data", multi_bool, self.scanner2_safe_callback)
        self.vel_sub = rospy.Subscriber("/cmd_vel", lust_cmd, self.vel_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odometry_callback)
        # TODO FIX NAMES
        self.vision_status_pub = rospy.Publisher("/platformStatus", Bool, queue_size=10)
        # TODO FIX COMMENTS
        self.vision_status_sub = rospy.Subscriber('/visionStatus', Bool, self.vision_callback)

        self.ctrl_c = False
        self.rate = rospy.Rate(10)

        self.fw_first_odom_read = False
        self.fw_running = False

        self.odom = Odometry()
        self.odom_start = 0
        self.odom_middle = 0
        self.end_flag = False

        # pausing movement parameters
        self.odom_pause = 0
        self.pausing_distance = 500 # in mm
        self.flag_pause = 999
        self.sensor_start = False
        self.sensor_done = True


        self.platform_status = 6 #Idle BLUE

        rospy.on_shutdown(self.shutdownhook)

    def scanner1_safe_callback(self, data):
        self.front_danger_thin = data.front_danger_thin
        self.front_danger_right = data.front_danger_right
        self.front_danger_wide = data.front_danger_wide
        self.front_alert = data.front_alert
        self.front_alert_right = data.front_alert_right
    
    def scanner2_safe_callback(self, data):
        self.back_danger_thin = data.back_danger_thin
        self.back_danger_left = data.back_danger_left
        self.back_danger_wide = data.back_danger_wide
        self.back_alert = data.back_alert
        self.back_alert_left = data.back_alert_left

    def vision_callback(self, data):
        self.sensor_done = data

    def vel_callback(self, msg):
        self.joy_vel = deepcopy(msg)
        self.vel_msg = msg

    def odometry_callback(self, odo_send):
        self.odom = odo_send

    def _check_laser1_ready(self):
        self.laser_msg1 = None
        rospy.loginfo(Fore.LIGHTYELLOW_EX + "Checking Sick NanoScan3 laser..." + Style.RESET_ALL)
        while self.laser_msg1 is None and not rospy.is_shutdown():
            try:
                self.laser_msg1 = rospy.wait_for_message("/scanner1_data", multi_bool, timeout=1.0)
                rospy.logdebug(Fore.LIGHTYELLOW_EX + "Current /scanner1_data READY=>" + str(self.laser_msg1) + Style.RESET_ALL)

            except:
                rospy.logerr(Fore.LIGHTYELLOW_EX + "Current /scanner1_data not ready yet, retrying for getting scan" + Style.RESET_ALL)
        rospy.loginfo(Fore.LIGHTYELLOW_EX + "Checking SICK Nanoscan3 1...DONE" + Style.RESET_ALL)
        return self.laser_msg1

    def _check_laser2_ready(self):
        self.laser_msg2 = None
        rospy.loginfo(Fore.YELLOW+"Checking Sick NanoScan3 laser..."+Style.RESET_ALL)
        while self.laser_msg2 is None and not rospy.is_shutdown():
            try:
                self.laser_msg2 = rospy.wait_for_message("/scanner2_data", multi_bool, timeout=1.0)
                rospy.logdebug(Fore.YELLOW + "Current /scanner2_data READY=>" + str(self.laser_msg2) + Style.RESET_ALL)
            except:
                rospy.logerr(Fore.YELLOW + "Current /scanner2_data not ready yet, retrying for getting scan"+ Style.RESET_ALL)
        rospy.loginfo(Fore.YELLOW + "Checking SICK Nanoscan3 2...DONE" + Style.RESET_ALL+"\n")
        rospy.loginfo(Fore.LIGHTGREEN_EX + "The Platform is READY!" + Style.RESET_ALL+"\n")
        return self.laser_msg2

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        self.stop_robot()


    ############ Stop functions ############################################################
    def stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        self.sensor_done = True
        
        # self.pub3.publish(self.platform_status)
        
        
        self.platform_status = 0 #Stop RED
        self.pub.publish(self.vel_msg)   



    def manual_stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        self.sensor_done = True
        print(Fore.WHITE + "\n###########################")
        print(           "#" + 25*" " + "#")
        print(           "#  " + Back.RED + Fore.WHITE + " STOP button pressed "+  Style.RESET_ALL + Fore.WHITE + "  #")
        print(Fore.WHITE + "#" + 25*" " + "#")        
        print(           "###########################\n"+ Style.RESET_ALL)
        self.platform_status = 0 #Stop RED        
        self.pub.publish(self.vel_msg)
        # Tell vision that platform is not moveing
        self.vision_status_pub.publish(True)

    def auto_stop_robot(self):
        self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
        #self.pub3.publish(self.platform_status)
        self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        self.pub.publish(self.vel_msg)
        # Tell vision that platform is not moveing
        self.vision_status_pub.publish(True)


    ############ Slowdown and idle functions ############################################################
    def slowdown_alert(self):
        self.vel_msg.cmd_vel.linear.x = self.joy_vel.cmd_vel.linear.x*0.5
        self.vel_msg.cmd_vel.angular.x = self.joy_vel.cmd_vel.angular.x*0.5
        self.vel_msg.cmd_vel.angular.z = self.joy_vel.cmd_vel.angular.z*0.5
        #self.pub3.publish(self.platform_status)
        self.platform_status = 1 #Alert signal YELLOW        
        self.pub.publish(self.vel_msg)

    def idle(self):
        self.vel_msg.cmd_vel.linear.x = 0
        self.vel_msg.cmd_vel.angular.x = 0
        self.vel_msg.cmd_vel.angular.z = 0
        #self.pub3.publish(self.platform_status)
        self.platform_status = 6 #Idle BLUE       
        self.pub.publish(self.vel_msg)

    ############ Varnost ob premikih naravnost naprej ############################################################
    def forward_movement(self):
        #print("forward")
        if (self.front_danger_right == True) or (self.back_danger_left == True) or (self.front_danger_wide == True) or (self.vel_msg.stop_run == True):
            print(Fore.LIGHTBLUE_EX+"\n[ Sensor1 " + Fore.RED + "Danger" + Fore.LIGHTBLUE_EX+" close! ]" + Style.RESET_ALL)
            self.stop_robot()
        elif ((self.front_danger_wide == True) and (self.front_alert == True)) or (self.vel_msg.stop_run == True):
            print(Fore.LIGHTBLUE_EX+"\n[ Sensor1 " + Fore.RED + "Danger" + Fore.LIGHTBLUE_EX+" close! ]" + Style.RESET_ALL)
            self.stop_robot()    
        elif self.front_alert == True:
            self.slowdown_alert()
        else:
            self.platform_status = 4 #Joystick control GREEN
            self.pub.publish(self.vel_msg) 

    ############  Varnost ob premikih naravnost nazaj ############################################################
    def backward_movement(self):
        #print("backward")
        if (self.front_danger_right == True) or (self.back_danger_left == True) or (self.back_danger_wide == True) or (self.vel_msg.stop_run == True):
            print(Fore.LIGHTCYAN_EX+"\n[ Sensor2 " + Fore.RED + "Danger" + Fore.LIGHTCYAN_EX + " close! ]" + Style.RESET_ALL)
            self.stop_robot()
        elif ((self.back_danger_wide == True) and (self.back_alert == True)) or (self.vel_msg.stop_run == True):
            print(Fore.LIGHTCYAN_EX+"\n[ Sensor2 " + Fore.RED + "Danger" + Fore.LIGHTCYAN_EX + " close! ]" + Style.RESET_ALL)
            self.stop_robot() 
        elif self.back_alert == True:
            self.slowdown_alert()
        else:
            self.platform_status = 4 #Joystick control GREEN
            self.pub.publish(self.vel_msg)  

    ############ Varnost ob zavijanju in vrtenju ############################################################
    def rotational_movement(self):
        #print("rotational")
        if (self.back_danger_wide == True) or (self.front_danger_wide == True) or (self.front_danger_right == True) or (self.back_danger_left == True) or (self.vel_msg.stop_run == True):
            self.stop_robot()
        else:    
            if (self.back_alert == True) or (self.back_alert_left == True) or (self.front_alert == True) or (self.front_alert_right == True):
                self.slowdown_alert()
            else:
                self.platform_status = 4 #Joystick control GREEN
                self.pub.publish(self.vel_msg)    

    ############ Pocasnejsa voznja naravnost in nazaj (upostevanje senzorjev) ####################################
    def slower_auto_movement_forward(self):
        if self.front_danger_thin == False:
            self.vel_msg.cmd_vel.linear.x = 0.75
            self.vel_msg.cmd_vel.angular.x = 0
            self.vel_msg.cmd_vel.angular.z = 0
            self.platform_status = 2 #Auto forward VIOLET
            #self.pub3.publish(self.platform_status)
            self.pub.publish(self.vel_msg)
            # Tell vision that platform is moveing
            self.vision_status_pub.publish(False)
        elif self.front_danger_thin == True or self.vel_msg.stop_run == True:
            print(Fore.LIGHTBLUE_EX+"\n[ Sensor1 " + Fore.RED + "Danger" + Fore.LIGHTBLUE_EX + " close! ]" + Style.RESET_ALL)
            self.flag = 999
            self.stop_robot()

    def slower_auto_movement_backward(self):
        if self.back_danger_thin == False:
            self.vel_msg.cmd_vel.linear.x = -0.75
            self.vel_msg.cmd_vel.angular.x = 0
            self.vel_msg.cmd_vel.angular.z = 0.0
            self.platform_status = 2 #Auto forward VIOLET
            self.pub.publish(self.vel_msg)
            # Tell vision that platform is moveing
            self.vision_status_pub.publish(False)
        elif self.back_danger_thin == True or self.vel_msg.stop_run == True:
            print(Fore.LIGHTCYAN_EX+"\n[ Sensor2 " + Fore.RED + "Danger" + Fore.LIGHTCYAN_EX + " close! ]" + Style.RESET_ALL)
            self.flag = 999
            self.stop_robot()

    ############ Override safety movement (no sensor data used) #####################################################
    def manual_override_safety_movement(self):
        print(Fore.LIGHTRED_EX+ "[ Override safety! ]")
        self.vel_msg.cmd_vel.linear.x = self.joy_vel.cmd_vel.linear.x*0.5
        self.vel_msg.cmd_vel.angular.x = self.joy_vel.cmd_vel.angular.x*0.5
        self.vel_msg.cmd_vel.angular.z = self.joy_vel.cmd_vel.angular.z*0.5
        #self.pub3.publish(self.platform_status)
        self.platform_status = 5 #Override joystick control BLINKING RED        
        self.pub.publish(self.vel_msg)


    ############ Funkcija po pretipkani kodi ########################################################################
    def safety_auto_mvmnt(self):
        if self.fw_first_odom_read:
            self.odom_start = self.odom.pose.pose.position.x
            self.fw_first_odom_read = False
            self.fw_running = True
        self.flag = 0
        if self.front_danger_thin == True:
            self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
            self.flag = 1   
            if not self.safety_signal_old.data:
                self.time0 = rospy.Time.now()
            
            dt = rospy.Time.now() - self.time0

            if dt.to_sec() > 5:
                self.flag = 2

        self.odom_middle = self.odom.pose.pose.position.x        
        #print("\nOdometrija ob pritisku tipke: " + Fore.LIGHTYELLOW_EX + str(self.odom_start) + Fore.RESET)
        #print("Odometrija ob vracanju nazaj: ", Fore.LIGHTGREEN_EX + str(self.odom_middle) + Fore.RESET, end='\n')
        if self.flag_old == 2 or self.flag_old == 3:
            self.flag = 2
            if ((self.odom_middle - self.odom_start) <= 0):
                self.flag = 999 
                self.fw_running = False
                self.end_flag = True
            elif (self.back_danger_thin == True):
                self.flag = 999
                self.flag = 3 
                self.platform_status = 3 #Obstacle AUTO Blinking YELLOW
        else:
            self.end_flag = True
            self.flag = self.flag

        self.safety_signal_old.data = self.front_danger_thin
        self.flag_old = self.flag
    
    ############ Avtonomni premiki s pavzami ############################################################
    #def safety_pausing_auto_mvmnt(self):

        '''elif (self.vel_msg.pause_run = True and self.field1_danger == False):
                        self.pausing_movement()'''
                        
    def safety_pausing_auto_movement(self):
        # read the odometry
        if self.fw_first_odom_read:
            self.odom_start_pause = self.odom.pose.pose.position.x
            self.fw_first_odom_read = False
            self.fw_running = True
            self.odom_pause = 0.0
            #self.flag_pause = 1
        
        # preveri, ce si dobil info iz senzorja, da je koncal
        # print(self.sensor_done)
        if self.sensor_done:
            self.sensor_start = False
            self.flag_pause = 1

            # print("Odom: " + str(self.odom.pose.pose.position.x))
            # print("Odom start: " + str(self.odom_start_pause))
            # print("Odom pause: " + str(self.odom_pause))
            # print((self.odom.pose.pose.position.x-self.odom_start_pause))

            # check if the distance achieved 
            if ((self.odom.pose.pose.position.x-self.odom_start_pause) -self.odom_pause) >= self.pausing_distance:
                rospy.loginfo("Naredili odometrijo, zacni skeniranje.")
                self.flag_pause = 2
                # ponastavi zadnjo znano pozicijo
                self.odom_pause = self.odom.pose.pose.position.x-self.odom_start_pause
                # poslji ustrezen signal, da se je platforma ustavila
                self.sensor_start = True
                # sensor_done mora iti na False!
                self.sensor_done = False
                self.vision_status_pub.publish(self.sensor_start) 











    ############ Premiki s kontrolerjem (upostevanje stop tipke) ############################################################
    def manual_movement(self):
        if self.vel_msg.stop_run == True:
            self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
            self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
            self.platform_status = 0 #Stop RED 
            self.pub.publish(self.vel_msg)
        else:
            self.platform_status = 4 #Joystick control GREEN
            #self.pub3.publish(self.platform_status)
            self.pub.publish(self.vel_msg) 


    ############ MAIN function ####################################################################################
    def ctrl_to_communication(self):
            while not self.ctrl_c:
                if not self.fw_running:
                    self.fw_first_odom_read = True

                if (self.vel_msg.start_run == True) and (self.flag == 999):
                    self.stop_robot()
                elif (self.vel_msg.start_run == True) and (self.flag != 999):
                    self.safety_auto_mvmnt()
                    if self.flag == 0:
                        self.slower_auto_movement_forward()
                    if self.flag == 1:
                        self.auto_stop_robot()
                    if self.flag == 2:
                        self.slower_auto_movement_backward()
                    if self.flag == 3:
                        self.auto_stop_robot()
                
                # start pausing run
                # elif (self.vel_msg.pause_run == True) and (self.flag_pause == 999):
                    # self.stop_robot()
                elif (self.vel_msg.pause_run == True): #and (self.flag_pause != 999):  
                    self.safety_pausing_auto_movement()
                    if self.flag_pause == 1:
                        self.slower_auto_movement_forward()
                    if self.flag_pause == 2:
                        self.auto_stop_robot()

                elif (self.vel_msg.stop_run == True):
                    self.flag = 0
                    self.odom_start = self.odom.pose.pose.position.x
                    self.manual_stop_robot()

                elif self.vel_msg.reset_odometry == True:
                    self.vel_msg.cmd_vel.linear = Vector3(0.0, 0.0, 0.0)
                    self.vel_msg.cmd_vel.angular = Vector3(0.0, 0.0, 0.0)
                    self.pub.publish(self.vel_msg)

                
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
                        self.pub.publish(self.vel_msg)
                    else:
                        self.idle()
                
                print(Style.RESET_ALL, end="\r")
                self.pub3.publish(self.platform_status)
                self.rate.sleep()


if __name__ == '__main__':

    lust_safe_control = Lust_safe_control()
    try:
        lust_safe_control.ctrl_to_communication()
    except rospy.ROSInterruptException:
        pass

