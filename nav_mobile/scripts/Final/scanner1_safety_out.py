#!/usr/bin/env python
from __future__ import print_function
import rospy
from sick_safetyscanners.msg import RawMicroScanDataMsg as IDM
from geometry_msgs.msg import Vector3, Twist
from lust_msgs.msg import multi_bool
from std_msgs.msg import Bool

#pub = rospy.Publisher("nanoscan3_safety", Bool, queue_size=10)


def callback(data):
    msg = multi_bool()
    msg.back_danger_thin = False
    msg.back_danger_left = False
    msg.back_danger_wide = False
    msg.back_alert = False
    msg.back_alert_left = False

    msg.front_danger_thin = False
    msg.front_danger_right = False
    msg.front_danger_wide = False
    msg.front_alert = False
    msg.front_alert_right = False

    area_front_danger_thin = 0
    area_front_danger_right = 0
    area_front_danger_wide = 0
    area_front_alert = 0
    area_front_alert_right = 0
    twist = Twist()

    intrusion_area1 = data.intrusion_data.data[0]
    intrusion_area2 = data.intrusion_data.data[1]
    intrusion_area3 = data.intrusion_data.data[2]
    intrusion_area4 = data.intrusion_data.data[3]
    intrusion_area5 = data.intrusion_data.data[4]

    intrusion_flags1 = intrusion_area1.flags
    intrusion_flags2 = intrusion_area2.flags
    intrusion_flags3 = intrusion_area3.flags
    intrusion_flags4 = intrusion_area4.flags
    intrusion_flags5 = intrusion_area5.flags



    for flag1 in intrusion_flags1:
            if flag1 == False:
                area_front_danger_right = area_front_danger_right
            elif flag1 == True:
                area_front_danger_right = area_front_danger_right +1
            else:
                continue

    for flag2 in intrusion_flags2:
        if flag2 == False:
            area_front_danger_thin = area_front_danger_thin
        elif flag2 == True:
            area_front_danger_thin = area_front_danger_thin + 1 
        else:
            continue

    for flag3 in intrusion_flags3:
        if flag3 == False:
            area_front_danger_wide = area_front_danger_wide
        elif flag3 == True:
            area_front_danger_wide = area_front_danger_wide + 1 
        else:
            continue

    for flag4 in intrusion_flags4:
        if flag4 == False:
            area_front_alert = area_front_alert
        elif flag4 == True:
            area_front_alert = area_front_alert + 1 
        else:
            continue

    for flag5 in intrusion_flags5:
        if flag5 == False:
            area_front_alert_right = area_front_alert_right
        elif flag5 == True:
            area_front_alert_right = area_front_alert_right + 1 
        else:
            continue
    
    if (area_front_alert > 3):
        if (area_front_danger_wide > 3) and (area_front_danger_thin <= 3):
            msg.front_danger_wide = True
            msg.front_danger_thin = False
            msg.front_alert = True
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
                #pub.publish(msg)
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
        #pub.publish(msg)
        elif (area_front_danger_thin > 3):
            msg.front_danger_wide = True
            msg.front_danger_thin = True
            msg.front_alert = True
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
                #pub.publish(msg)
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
        elif (area_front_danger_wide < 3):
            msg.front_danger_wide = False
            msg.front_danger_thin = False
            msg.front_alert = True
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
                #pub.publish(msg)
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
    elif (area_front_alert <= 3):
        if (area_front_danger_wide <= 3) and (area_front_danger_thin <= 3):
            msg.front_danger_wide = False
            msg.front_danger_thin = False
            msg.front_alert = False
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
        elif (area_front_danger_thin > 3):
            msg.front_danger_wide = True
            msg.front_danger_thin = True
            msg.front_alert = False
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
                #pub.publish(msg)
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
        elif (area_front_danger_wide > 3) and (area_front_danger_thin <= 3):
            msg.front_danger_wide = True
            msg.front_danger_thin = False
            msg.front_alert = False
            if (area_front_alert_right >= 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = True
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = True
                    msg.front_danger_right = False
                #pub.publish(msg)
            if (area_front_alert_right < 3):
                if (area_front_danger_right >= 3):
                    msg.front_alert_right = False
                    msg.front_danger_right = True
                else:
                    msg.front_alert_right = False
                    msg.front_danger_right = False
        pub.publish(msg)

    pub.publish(msg)
    


if __name__ == '__main__':
    rospy.init_node("nanoscan3_publish_safety", anonymous=True)
    sub = rospy.Subscriber("/scanner1/sick_safetyscanners/raw_data", IDM, callback)
    pub = rospy.Publisher("/scanner1_data", multi_bool, queue_size=10)

    rospy.spin()
