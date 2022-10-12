#!/usr/bin/env python

#Tests of TCP client
import keyboard
import rospy

def keyboardCode():
    # initialise ros node
    rospy.init_node('computerVision_comm_test', anonymous=True)
    while not rospy.is_shutdown:
        try:
            if keyboard.is_pressed('q'):
                print("pressed q")
        except:
            break

if __name__ == '__main__':

    try:
        keyboardCode()
        
    except rospy.ROSInterruptException:
        print("Platform-vision comm error")