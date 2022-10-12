#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from sick_safetyscanners.msg import RawMicroScanDataMsg as IDM





def callback(data):
    #rospy.loginfo(data)
    num_false1 = 0
    num_false2 = 0
    num_true1 = 0
    num_true2 = 0

    danger1 = False
    alert2 = False

    intrusion_area1 = data.intrusion_data.data[0]
    intrusion_area2 = data.intrusion_data.data[1]
    intrusion_flags1 = intrusion_area1.flags
    intrusion_flags2 = intrusion_area2.flags

    #print(intrusion_flags1)

    for flag in intrusion_flags1:
        if flag == False:
            area1 = "Safe"
            num_false1 = num_false1 + 1
        elif flag == True:
            area2 = "Not Safe"
            num_true1 = num_true1 +1 
        else:
            continue
    
    for flag in intrusion_flags2:
        if flag == False:
            area1 = "Safe"
            num_false2 = num_false2 + 1
        elif flag == True:
            area2 = "Not Safe"
            num_true2 = num_true2 +1 
        else:
            continue
    
    if num_true2 > 0:
        alert2 = True
        print("Area is obstructed!\n")
    elif num_true2 == 0 and num_true2 < 10:
        alert2 = False
        print("Area not obstructed!\n")
    if num_true1 > 0:
        danger1 = True
        print("Danger close!\n")
    elif num_true1 == 0 and num_true1 < 10:
        danger1 = False
        print("Area safe!\n")

    print(num_false)
    print(num_true)
    #rospy.loginfo(array2)


if __name__ == '__main__':
    rospy.init_node("nanoscan3_receive", anonymous=True)
    sub = rospy.Subscriber("/sick_safetyscanners/raw_data", IDM, callback)
    #data = IDM()
    rospy.spin()
