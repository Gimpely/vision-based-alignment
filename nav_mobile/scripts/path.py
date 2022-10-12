#!/usr/bin/env python
import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
#import tf
from sensor_msgs.msg import NavSatFix
import utm

from ublox_msgs.msg import NavPVT
import numpy as np 

global msg     
msg = Path()
msg2 = Path()


prviX = 0
prviY = 0

prviX_lonLat = 0
prviY_lonLat = 0

stSat = 0



def callbackGPSon(data):
    global stSat 
    podatkia = NavPVT()
    stSat = data.numSV
    #print(data.numSV)
    
def callback(data):
    
    global prviX , prviY , stSat,prviX_lonLat,prviY_lonLat
    # Assign the data to variables
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    covariance_type = data.position_covariance_type
    covariance = [data.position_covariance[0],data.position_covariance[4],data.position_covariance[8]]
    
    utm_data = utm.from_latlon(latitude, longitude)
    
     
    if prviX == 0 and prviY == 0:
        prviX = utm_data[0]
        prviY = utm_data[1]
        prviX_lonLat = latitude
        prviY_lonLat = longitude  
    # Print
    print("---------------")
    print("pos relative:")
    
    print("Latitude:", round(latitude-prviX_lonLat,5))
    print("Longitude:", round(longitude-prviY_lonLat,5))
    
    print("UTM-X:", round(utm_data[0]-prviX,4))
    print("UTM-Y:", round(utm_data[1]-prviY,4))
    #print("UTM-zone:", utm_data[2])
    
    
    #print("!status", data.status.status)
    #print("!service", data.status.service)
    #print("!SERVICE_GPS", data.status.SERVICE_GPS)
    print("STEVILO UPORABLJENIH SAT:", stSat)
    
    
    print("covariance:", np.around(covariance,3) )
    print("covariance_type:", covariance_type)


    
    
    # za UTM   
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.pose.position.x = prviX - utm_data[0]
    pose.pose.position.y = prviY - utm_data[1]
    pose.pose.position.z = 0

    msg.poses.append(pose)

    # za long lat 
    msg2.header.frame_id = "map"
    msg2.header.stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.pose.position.x = round(latitude-prviX_lonLat,5)
    pose.pose.position.y = round(longitude-prviY_lonLat,5)
    pose.pose.position.z = 0
    msg2.poses.append(pose)
    
    rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))     
    pub.publish(msg)
    pub2.publish(msg2)
    

if __name__ == '__main__':
    try:
        
        pub = rospy.Publisher('UTM', Path, queue_size=10)
        pub2 = rospy.Publisher('LonLAT', Path, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rospy.Subscriber('/gps/fix', NavSatFix, callback)
        rospy.Subscriber('/gps/navpvt', NavPVT, callbackGPSon)
        rospy.spin()

        
    except rospy.ROSInterruptException:
        pass
    
    

