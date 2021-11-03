#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import time

def logdata_cb(msg):
    file.write(msg.data + "\n")

#def logdata2_cb(msg):
 #   file2.write(str(time.time()*1000) + "," + str(msg.data[1]) + "\n")
    

if __name__ == '__main__':

    rospy.init_node('TeensyLOG')

    fileName = 'AnotherSent_DataV2.csv'
    #fileName2 = 'Recievedv3_Data.csv'
    file = open(fileName,"w")
    #file2 = open(fileName2,"w")

    sub_logdata = rospy.Subscriber('LOGDATA',String,logdata_cb,queue_size=1)
    #sub_logdata2 = rospy.Subscriber('/simple_hexapod/Th2_1_position_controller/command',Float64MultiArray,logdata2_cb,queue_size=1)

    while not rospy.is_shutdown():
        rospy.sleep(1)

    file.close()
    #file2.close()
    print('done')