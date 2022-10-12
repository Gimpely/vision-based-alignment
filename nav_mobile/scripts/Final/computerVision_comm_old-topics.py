#!/usr/bin/env python
import rospy
import numpy as np
import socket
import struct
import time as t
from datetime import datetime as dt
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32

# ROS node for communicating with computer vision system via TCP client

TCP_PORT = 53010 # Port to send data to
TCP_IP = "10.8.0.60" # Adress to send data to (vision system)

class TCPClient():
    def __init__(self):
        # initialise ros node
        rospy.init_node('computerVision_comm', anonymous=True)
        # Init variables
        self.rate=rospy.Rate(10) #10Hz
        self.odom = Odometry()
        self.odom_start = Odometry()
        self.program = 0
        self.moveingFlag = True
        self.posX = 0.0
        self.prog2running = False
        self.myPath = '/home/staublixpc/moveNScan_logs/'
        self.myFileType = '.log'

        ### Testing #####################################################################
        self.program = 2

        # Subscribe to odometry topic
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odometry_callback)
        # Subscribe to programSelect topic
        self.prog_sub = rospy.Subscriber("/programSelect", Int32, self.prog_callback)
        # Subscribe to platformStatus topic
        self.moveing_sub = rospy.Subscriber("/platformStatus", Bool, self.status_callback)
        # Create publisher for scanDone
        self.statusPub = rospy.Publisher('/visionStatus', Bool, queue_size=10)
    
    ### CALLBACKS #######################################################################
    def odometry_callback(self, odo_topic):
        self.odom = odo_topic
        self.posX = self.odom.pose.pose.position.x
        # print("Debug: posX = " + str(self.posX))

    def prog_callback(self, prog_topic):
        # 0 = default, 1 = Stop&go, 2 = Move&Scan
        self.program = prog_topic
        print("Prog callback")

    def status_callback(self, status_topic):
        # False = not moveing, True = moveing
        # print("Debug: moveing " + str(status_topic.data))
        self.moveingFlag = not status_topic.data
        # If program 2 (move&scan)...
        # print("Debug: moveing " + str(self.moveingFlag))
        if self.program == 2:
            self.move_n_scan()
        # print("status callback")

    ### FUNCTIONS #######################################################################

    def to_bytes(self, n, length, endianess='big'):
        h = '%x' % n
        s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
        return s if endianess == 'big' else s[::-1]

    def from_bytes(self, n):
        # Assumes a big endian int32 (long)
        intArray = struct.unpack('>l',n)
        myInt = intArray[0]
        return myInt

    def validate_packet(self, packet, original):
        valid = False
        # Unpack packet
        try:
            # packet = struct.unpack('>1f',packet)
            # packet = packet[0]
            packet = self.from_bytes(packet)
            print("Debug: msg received: " + str(packet))
            # Check if same packet as sent
            # original = struct.unpack('>1f',original)
            # original = original[0]
            original = self.from_bytes(original)
            if packet == original:
                valid = True
                # print("Debug: Valid")
        except:
            valid = False

        return valid

    def stop_n_go(self):
        # Communication for the stop&go program
        # print("Debug: Inside stop_n_go")
        # print("Debug: moveingFlag = " + str(self.moveingFlag))

        # If platform has halted...
        if not self.moveingFlag:
            # msg = struct.pack('>1f',self.posX)
            posX_int = np.int32(round(self.posX))
            msg = self.to_bytes(posX_int, 4, 'big')
            # print("Debug: msg send = "+ str(msg))
            self.sock.sendall(msg)
            print("TCP send: posX")
            # Read socket for data, blocking
            data = self.sock.recv(1024)
        
            # If valid response...
            if self.validate_packet(data,msg):
                # Resume movement, True = done
                self.statusPub.publish(True)
                print("Published status: True = Done")
            
            # Reset flag to prevent unwanted resend
            self.moveingFlag = True

    def move_n_scan(self):
        # Communication for the move&scan program
        # If platform has started moveing
        # print("Debug: Inside move_n_scan")
        # print("Debug: moveingFlag = " + str(self.moveingFlag))
        if self.moveingFlag and not self.prog2running:
            # msg = struct.pack('>1f',9999.0)
            msg = self.to_bytes(1999999999, 4, 'big')
            self.prog2running = True
            tmp = "1999999999"
            self.sock.sendall(msg)
            print("Debug: TCP send " + tmp)
        elif not self.moveingFlag and self.prog2running:
            # msg = struct.pack('>1f',10000.0)
            msg = self.to_bytes(2000000000, 4, 'big')
            self.prog2running = False
            tmp = "2000000000"
            self.sock.sendall(msg)
            print("Debug: TCP send " + tmp)

    def startClient(self):
        # Configure a free socket for TCP/IP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect
        self.sock.connect((TCP_IP, TCP_PORT))
        rospy.loginfo("TCP to vision opened")
        # posX_int = np.int32(round(self.posX))
        # msg = self.to_bytes(1234, 4, 'big')
        # # print("Debug: msg send = "+ str(msg))
        # self.sock.sendall(msg)
        
        # Init epoch time
        time1 = 0.0
        # Main Loop
        while not rospy.is_shutdown():
            # Select program
            # Prog2 (move_n_scan) is not looped, triggers on platformStatus callback if enabled
            if self.program == 1:
                self.stop_n_go()
            
            elif self.program == 2 and self.prog2running:
                time2 = t.time()
                # If more than 0.5 have passed...
                if time2-time1 > 0.5:
                    # Reset epoch time
                    time1 = t.time()
                    # Grab date-time
                    now = dt.now()
                    myDateTime = str(now.date()) + '-' + str(now.hour) + '-' + str(now.minute) + '-' + str(now.second) + '-' + str(int(round(now.microsecond/1e6,2)*1e2))
                    # write date-time and odometry to file
                    with open(self.myPath + myDateTime + self.myFileType,'w') as logfile:
                        logfile.write(str(int(round(self.posX))))
                        # print('Debug: Writing')

            
            # Loop at rate
            self.rate.sleep()
        self.sock.close()
        rospy.loginfo("TCP to vision closed")


if __name__ == '__main__':

    TCP_Client = TCPClient()
    try:
        rospy.loginfo("Init platform-vision comm")
        TCP_Client.startClient()
        
    except rospy.ROSInterruptException:
        print("Platform-vision comm error")
