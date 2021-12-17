#!/usr/bin/env python       

import rospy                                                                 #ROS-library
import numpy as np                                                           #zur Arrayerstellung
from std_msgs import msg                                                              
from std_msgs.msg import Int16                                              
from sensor_msgs.msg import NavSatFix                                      #imports the data type of the GPS-Sensor's topic / message
from geometry_msgs.msg import Twist 
#from test.msg import signals 
#keine Bewegung = 0 vorwärts = 1 rückwärts = 2 rechts = 3 links = 4

#status= signals()

#mode = 0
pub = rospy.Publisher('steuerung', Int16, queue_size = 10)                  #topic called "steuerung" has been created here. The queue is being held in case the publishing frequency is higher than the loop on the subscriber's side
mode = Int16()
mode = 0


def fahrtrichtung(msg):
    global mode

    rospy.loginfo("Fahrtichtung fkt called")
    if msg.angular.z > 0:                           #rechts
        mode = 3
        #pub.publish(mode) 
    elif msg.angular.z < 0:                          #links
                                     
        mode = 4
        #pub.publish(mode)
    elif msg.linear.x < 0:                           #rückwärts
        mode = 2
    elif msg.linear.x > 0:
        mode = 1
    elif msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:   #klappt noch nicht
        mode = 0
    
rospy.loginfo(mode)

def gps_signal(msg):          
    rospy.loginfo("gps_signal fkt called")                                            #Aufruffunktion des Publishers
    if msg.latitude != 0 or msg.longitude != 0 or msg.altitude != 0:
        #status.GPS.data = 1                                                 # Wert der Message wird festgelegt
        global mode 
        #rospy.loginfo("Mode " + mode) 
        if(mode == 0): mode = 10
        elif(mode == 1): mode = 11
        elif(mode == 2): mode = 12
        elif(mode == 3): mode = 13
        elif(mode == 4): mode = 14
        pub.publish(mode)

        #pub.publish(status)                                                #der Message wird gepublished
    else:
        #status.GPS.data = 0                                                # Wert der Message wird festgelegt
        if(mode == 0): mode = 20
        elif(mode == 1): mode = 21
        elif(mode == 2): mode = 22
        elif(mode == 3): mode = 23
        elif(mode == 4): mode = 24
        pub.publish(mode)                                                #der Message wird gepublished
   
def main():                                                               #Aufruffunktion des Suscribers
    rospy.init_node('status_signal', anonymous=True)                        
                                            
    sub = rospy.Subscriber('/twist_mux/cmd_vel', Twist, fahrtrichtung)       #Suscribt zu Rostopic fix
    sub = rospy.Subscriber('/ublox/fix', NavSatFix, gps_signal)       #Suscribt zu Rostopic fix
    rate = rospy.Rate(15)

    rospy.loginfo("Sucessfully started node")  

    while not rospy.is_shutdown():
        rate.sleep()
        # pub.publish(mode)
main()





