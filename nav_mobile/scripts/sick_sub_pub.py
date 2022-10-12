#!/usr/bin/env python
from __future__ import print_function
import rospy
from sick_safetyscanners.msg import RawMicroScanDataMsg as IDM
from geometry_msgs.msg import Vector3, Twist
from lust_msgs.msg import multi_bool
from std_msgs.msg import Bool

#pub = rospy.Publisher("nanoscan3_safety", Bool, queue_size=10)

global danger1
global alert2

def callback(data):
    msg = multi_bool()
    msg.field1_alert = False
    msg.field1_danger = False
    msg.field2_alert = False
    msg.field2_danger = False

    alert_field1 = 0
    danger_field1 = 0
    twist = Twist()

    intrusion_area1 = data.intrusion_data.data[1]
    intrusion_area2 = data.intrusion_data.data[0]
    intrusion_flags1 = intrusion_area1.flags
    intrusion_flags2 = intrusion_area2.flags

    for flag2 in intrusion_flags2:
        if flag2 == False:
            alert_field1 = alert_field1
        elif flag2 == True:
            alert_field1 = alert_field1 + 1 
        else:
            continue

    for flag1 in intrusion_flags1:
        if flag1 == False:
            danger_field1 = danger_field1
        elif flag1 == True:
            danger_field1 = danger_field1 +1
        else:
            continue
        
    print(alert_field1)
    print(danger_field1)

    
    if (alert_field1 > 10):
        msg.field1_alert = True
        msg.field1_danger = False
        #msg.field2_alert = False
        #msg.field2_danger = False
        #pub.publish(msg)
        #print("Area1 danger close!")
    elif (alert_field1 <= 10):
        msg.field1_alert = False
        msg.field1_danger = False
        #msg.field2_alert = False
        #msg.field2_danger = False
        #pub.publish(msg)

    if (danger_field1 > 3):
        msg.field1_alert = False
        msg.field1_danger = True
        #msg.field2_alert = False
        #msg.field2_danger = False

    elif ((danger_field1 > 3) and (alert_field1<=10)):
        msg.field1_alert = False
        msg.field1_danger = True
        #msg.field2_alert = False
        #msg.field2_danger = False
        #pub.publish(msg)
        #print("sickTrue\n")

    if (danger_field1 > 3):
        msg.field1_alert = False
        msg.field1_danger = True
        #msg.field2_alert = False
        #msg.field2_danger = False
        
    elif ((danger_field1 <= 3)  and (alert_field1 <= 10)):
        msg.field1_alert = False
        msg.field1_danger = False
        #msg.field2_alert = False
        #msg.field2_danger = False
        #pub.publish(msg)
        #print("Area1 safe!")
        #print("Area2 not obstructed!\n")


    pub.publish(msg)
    print(msg.field1_alert)
    print(msg.field1_danger)


if __name__ == '__main__':
    rospy.init_node("nanoscan3_publish_safety", anonymous=True)
    sub = rospy.Subscriber("/scanner1/sick_safetyscanners/raw_data", IDM, callback)
    pub = rospy.Publisher("/sick_sub_pub", multi_bool, queue_size=10)

    rospy.spin()
