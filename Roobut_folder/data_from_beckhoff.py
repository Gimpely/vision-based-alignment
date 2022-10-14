#!/usr/bin/env python
import math 
from math import sin, cos, asin, acos, atan2
import rospy
import socket
import struct
import binascii
import time
#from std_msgs.msg import String
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3

import tf

import time, math

global seq

seq = 0

UDP_PORT = 11312
UDP_IP = "192.168.65.57"

def recieve_packet(s):
    end = False
    packet = []
    while True:
        dataArr, addr = s.recvfrom(40)
        #data = dataArr[0]
        #rospy.loginfo(str(data))
        #rospy.loginfo(dataArr)
        #rospy.loginfo(len(dataArr))
        packet = list(dataArr)
        
        """if str(data) == 'T' and end:
            return packet
        if str(data) == 'T':
            end = True
        else:
            end = False"""
        return packet

def decode_packet(packet):
    valid = False
    odom_data = {}
  #  rospy.loginfo(len(packet))
    #if len(packet) == 40 and
    if (str(packet[0]) == 'S' and str(packet[1]) == 'S' and str(packet[-1]) == 'T' and str(packet[-2]) == 'T'):
        valid = True
       # rospy.loginfo("valid")
    if valid:
        i = 2
        odom_data["vel_L"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["vel_R"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["lin_vel"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["ang_vel"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["x"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["y"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
        odom_data["th/pi"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
        i = i + 4
    return valid, odom_data
            

def talker():
    rospy.loginfo("V funkciji talker.")
    #definiran publisher s Topic: "odo_info", oddaja informacijo tipa Vector3
    odo_info = rospy.Publisher("odo_info", Vector3, queue_size=10)
    #definiran Publisher s Topic: "odom", oddaja informacije in podatke o odometriji
    pub = rospy.Publisher("odom", Odometry, queue_size=10)
    #Dodatni Filter za publishing
    pub_filtered = rospy.Publisher("odometry/filtered", Odometry, queue_size=10)


    
    info = Vector3()


    global seq
    #pub = rospy.Publisher('odometry_1', Odometry, queue_size=10)
    #definiran node inicializacije z imenom odjemalec
    rospy.init_node('odjemalec', anonymous=True)
    tfBr =tf.TransformBroadcaster()
    rate=rospy.Rate(10000) #10Hz

    #udp comm stuff
    global sock
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    while not rospy.is_shutdown():
        Hello_str = "Hello world %s" %rospy.get_time()
        start_time = time.time()
        packet = recieve_packet(sock)
        valid, odom_data = decode_packet(packet)

        if valid:
          
            #rospy.loginfo_throttle(1, str(odom_data))
            #rospy.loginfo(str(odom_data["th/pi"]))
            info.x = float(odom_data["x"])
            info.y = float(odom_data["y"]) 
            info.z = float(odom_data["th/pi"])
            #odo_info.publish(info.y)


#   NAV STACK

            point = (odom_data["x"], odom_data["y"], 0)
            rot = (0, 0, 1 * math.sin(odom_data["th/pi"]/2), 1 * math.cos(odom_data["th/pi"]/2))
            tfBr.sendTransform(point, rot, rospy.Time.now(), "base_link", "odom")

            odo_send = Odometry()

            odo_send.header.seq = seq
            time_temp = time.time()
            odo_send.header.stamp.secs = math.floor(time_temp)
            odo_send.header.stamp.nsecs = math.floor(1000000000 * (time_temp - math.floor(time_temp)))
            odo_send.header.frame_id = "odom"
            odo_send.child_frame_id = "base_link"



            odo_send.pose.pose.position.x = odom_data["x"]
            odo_send.pose.pose.position.y = odom_data["y"]
            

            odo_send.pose.pose.orientation.x = 0
            odo_send.pose.pose.orientation.y = 0
            odo_send.pose.pose.orientation.z = 1 * math.sin(odom_data["th/pi"]/2)
            odo_send.pose.pose.orientation.w = 1 * math.cos(odom_data["th/pi"]/2)

            odo_send.pose.covariance =     [0.003, 0    , 0    , 0    , 0    , 0    ,
                             0    , 0.003, 0    , 0    , 0    , 0    ,
                             0    , 0    , 0.003, 0    , 0    , 0    ,
                             0    , 0    , 0    , 0.003, 0    , 0    ,
                             0    , 0    , 0    , 0    , 0.003, 0    ,
                             0    , 0    , 0    , 0    , 0    , 0.003]




            odo_send.twist.twist.linear.x = 0
            odo_send.twist.twist.linear.y = 0
            odo_send.twist.twist.angular.z = 0


            pub.publish(odo_send)
            pub_filtered.publish(odo_send)
            


if __name__ == '__main__':
    try:
        rospy.loginfo("Pred taklerjem")
        talker()
        
    except rospy.ROSInterruptException:
        print("ups")
