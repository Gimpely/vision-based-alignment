#!/usr/bin/env python

import rospy
from sick_safetyscanners.msg import IntrusionDatumMsg
from sick_safetyscanners.msg import IntrusionDataMsg
#rom ID import bool flags



def talker():
    nanoscan_pub = rospy.Publisher('nanoscan_talk', IntrusionDatumMsg, queue_size = 10)
    rospy.init_node('nanoscan_pub', anonymous=True)
    r = rospy.Rate(10)
    data = IntrusionDatumMsg()
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        nanoscan_pub.publish(data)
        r.sleep()





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
            pass