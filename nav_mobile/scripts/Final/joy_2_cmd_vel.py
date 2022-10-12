#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from lust_msgs.msg import lust_cmd

msg = lust_cmd()
global btn_
btn_ = False
global autostart
autostart = False



def autostart_callback(btn):
    global btn_
    global autostart
    
    if btn_ != btn and btn:
        #print("zanka")
        autostart = not autostart
        btn_ = btn
    #print("btn_ je " + str(btn_))
    return autostart


def callback(data):
    global btn_
    
    twist = Twist()
    if data.buttons[4] == 1.0:
        twist.linear.x = data.axes[1]
        #twist.angular.x = data.axes[1]
        twist.angular.z = data.axes[0]
    elif (data.buttons[4] != 1.0):
        twist.linear.x = 0
        twist.angular.x = 0
        twist.angular.z = 0
    #print(msg.start_run)
    if (data.buttons[0] == 1.0) and (data.buttons[1] == 1):
        #print("notr sm")
        msg.start_run  = autostart_callback(True)
        #msg.start_run = True
    else:
        btn_ = False

        #print(msg.start_run)
        #stop_run = True

    


    if data.buttons[2] == 1.0:
        twist.linear.x = 0
        twist.angular.x = 0
        twist.angular.z = 0
        msg.stop_run = True
        msg.start_run = False
        msg.override_safety = False
        msg.reset_odometry = False

    elif (data.buttons[3]) == 1.0:
        msg.reset_odometry = True

    elif data.buttons[5] == 1.0:
            msg.override_safety = True
            twist.linear.x = data.axes[1]
            #twist.angular.x = data.axes[2]
            twist.angular.z = data.axes[0]
        #print(msg)
        
    else:
        msg.reset_odometry = False
        msg.stop_run = False
        msg.override_safety = False

   
    
    msg.cmd_vel = twist
    pub.publish(msg)

 
def start():
    global pub
    global msg
    rospy.init_node('Controller_to_Send_data_node')

    pub = rospy.Publisher('/cmd_vel', lust_cmd, queue_size=1)
    rospy.Subscriber("/joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    start()