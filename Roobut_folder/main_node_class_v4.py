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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from copy import deepcopy, copy
from colorama import Fore,Back, Style
import os
import rospy
import numpy as np
import math
import open3d
import copy
import tf.transformations as tr


 

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
        
        # subscriber to Realsense camera
        # self.pointcloud_sub = rospy.Subscriber("/camera/depth/color/points",PointCloud2, self.pointcloud_callback)


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
        ##################################################### Added for pointcloud ########################
    
            
    def pointcloud_object(self, cloud_data):

        # Kot kamere od vertikale
        kot=23.5
        kot = np.radians(kot)
        # Ustvari pointlcoud iz podatkov kamere
        open3d_cloud = open3d.geometry.PointCloud()
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ]
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        # Ustvari eno tocko na poziciji kamere
        center_cloud = open3d.geometry.PointCloud()
        coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
        center_cloud.points = open3d.utility.Vector3dVector(coord)
        # Downsampling
        downpcd = open3d_cloud.voxel_down_sample(voxel_size=0.01)
        # Izluscenje cevi iz celotnega pointclouda
        points = np.asarray(downpcd.points)
        dists = downpcd.compute_point_cloud_distance(center_cloud)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists)
        cc = points[min_idx][2]
        c = downpcd.get_center()
        downpcd.translate((-c[0],-c[1],-c[2]))
        R = downpcd.get_rotation_matrix_from_axis_angle([-kot,0,0])
        downpcd.rotate(R, center=True)
        downpcd.translate((c[0],(c[1]),c[2]))
        points = np.asarray(downpcd.points)
        dists = downpcd.compute_point_cloud_distance(center_cloud)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists)
        downpcd.translate((0,-points[min_idx][1] + cc,0))
        ########################### visina kamere v metrih ################################
        #visina_kamere = 0.4
        visina_kamere = cc #+ 0.05
        ###############################################################################
        points = np.asarray(downpcd.points)
        cloud_Z = downpcd.select_down_sample(np.where(points[:,1] < visina_kamere)[0])
        points = np.asarray(cloud_Z.points)
        cloud_Z = cloud_Z.select_down_sample(np.where(points[:,2] < 1.8)[0])
        plane_model, inliers = cloud_Z.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        [a, b, c, d] = plane_model
        outlier_cloud = cloud_Z.select_down_sample(inliers, invert=True)
        cl, ind = outlier_cloud.remove_radius_outlier(nb_points=50, radius=0.08)
        new_cloud = outlier_cloud.select_down_sample(ind)
        return new_cloud
        

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)

        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.paint_uniform_color([1, 0.706, 0])

        source_temp.transform(transformation)
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
        open3d.visualization.draw_geometries([source_temp, target_temp, mesh])

    def preprocess_point_cloud(self, pcd, voxel_size): 
        pcd_down = pcd.voxel_down_sample(voxel_size)
        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        pcd_fpfh = open3d.registration.compute_fpfh_feature(pcd_down,open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        result = open3d.registration.registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
                open3d.registration.TransformationEstimationPointToPoint(False), 4, 
                [open3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),open3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
                open3d.registration.RANSACConvergenceCriteria(1000000, 600))
        return result

    def rotationMatrixToEulerAngles(self, R) :
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])
    
    def eulerToRotMatrix(self, y_kot):
        theta1 = 0
        theta2 = np.rad2deg(y_kot)
        theta3 = 0

        c1 = np.cos(theta1 * np.pi / 180)
        s1 = np.sin(theta1 * np.pi / 180)
        c2 = np.cos(theta2 * np.pi / 180)
        s2 = np.sin(theta2 * np.pi / 180)
        c3 = np.cos(theta3 * np.pi / 180)
        s3 = np.sin(theta3 * np.pi / 180)

        matrix=np.array([[c2*c3, -c2*s3, s2],
                        [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                        [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
        return matrix


    def findStart(self, source):

        origin = open3d.geometry.PointCloud()
        origin_coord = np.array([[0,0,0]]) 
        origin.points = open3d.utility.Vector3dVector(origin_coord)

        points = np.asarray(source.points)
        dists = source.compute_point_cloud_distance(origin)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists)
        z = points[min_idx][2]

        zz = z + 0.25
        start = source.select_down_sample(np.where(points[:,2] < zz)[0])

        return start

    def get_Transformation(self,source, target, voxel_size=0.02, debug=False, viz=False):
        crit = True

        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        cnt = 0
        while crit:
            cnt += 1
            result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
            print("Trying")
            if (result_ransac.inlier_rmse < 0.016) and (result_ransac.fitness > 0.4) : ### za 22.6 stopinj dej rmse<0.016 pa fit>0.4
                                                                                        ### za 35 stopinj dej rmse<0.016 pa fit>0.48
                rezult = result_ransac.transformation
                if viz:
                    self.draw_registration_result(source_down, target_down, result_ransac.transformation)
                T = np.linalg.inv(rezult)
                camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0])) 
                target.paint_uniform_color([0, 0, 1])
                camera.transform(np.linalg.inv(rezult))
                mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.array([0,0,0]))
                R = rezult[:3,:3]
                y_Eu = self.rotationMatrixToEulerAngles(R)[1]
                z_Eu = self.rotationMatrixToEulerAngles(R)[2]
                q = tr.quaternion_from_matrix(rezult)
                if viz:
                    open3d.visualization.draw_geometries([ camera, source,mesh])
                crit = False
                err = False
                cnt = 0
                return err, T, q
            elif cnt == 10:
                err = True
                T = 0
                q = 0
                return err, T, q

    def pointcloud_callback(self, data):
        cloud_data = list(pc2.read_points(data, field_names = ("x", "y", "z","rgb"), skip_nans=True))
        source = self.pointcloud_object(cloud_data, viz=False, debug=False)
        target = open3d.io.read_point_cloud("/home/staublixpc/catkin_ws/src/nav_mobile/lab_AGV-main/pcl_targets/target_pointcloud_2.pcd")
        short_target = open3d.io.read_point_cloud("/home/staublixpc/catkin_ws/src/nav_mobile/lab_AGV-main/pcl_targets/target_pointcloud_2_rob.pcd")
        voxel_size=0.02
        short_source = self.findStart(source)
        err, T, q = self.get_Transformation(short_source, short_target, voxel_size, debug=False, viz=False)
        if err:
            print("Failed")
            
        else:
            print("Done")
            print(T)




        #######################################################################################################
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

    # def _check_laser1_ready(self):
    #     self.laser_msg1 = None
    #     rospy.logdebug("Checking Sick NanoScan3 laser...")
    #     while self.laser_msg1 is None and not rospy.is_shutdown():
    #         try:
    #             self.laser_msg1 = rospy.wait_for_message("/scanner1_data", multi_bool, timeout=1.0)
    #             rospy.logdebug("Current /scanner1_data READY=>" + str(self.laser_msg1))
    #         except:
    #             rospy.logerr("Current /scanner1_data not ready yet, retrying for getting scan")
    #     rospy.loginfo("Checking SICK Nanoscan3 1...DONE")
    #     return self.laser_msg1

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

                # Take pointcloud
                elif self.vel_msg.take_pointcloud == True:

                    print("Hapopy days")




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

