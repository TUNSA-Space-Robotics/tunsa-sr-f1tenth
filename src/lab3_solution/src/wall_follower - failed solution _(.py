#!/usr/bin/env python

#Python Imports

from __future__ import print_function
import sys
from xml.sax.handler import DTDHandler
import numpy as np
import math

#ROS Imports
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import rospy
from sensor_msgs.msg import Image, LaserScan

#PID CONTROL PARAMS
kp = 2 #TODO 1
kd = 0.000 #TODO 0.001
ki = 0.005 #TODO 0.005

integral=0.0
prev_error=0.0
error=0.0

#WALL FOLLOW PARAMS
servo_offset = 0.0
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
VELOCITY = 2.00 # meters per second


class WallFollow:
    """ Implement Wall Following on the car
    """
    inda=0
    indb=0
    a_angle=0
    b_angle=0
    a_range=0
    b_range=0
    d_t=0.0
    d_t1=0.0
    drive_angle=0.0

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback,queue_size=1000)
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=1000)

    def getRange(self, lidar_info):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        b_angle=math.radians(180)
        a_angle=math.radians(135)
        indb=math.floor((b_angle-lidar_info.angle_min)/lidar_info.angle_increment)       

        if (lidar_info.angle_min > a_angle):
            a_angle = lidar_info.angle_min
            inda = 0
        else:
            inda = math.floor((a_angle-lidar_info.angle_min)/lidar_info.angle_increment)       
        if(not(np.isinf(lidar_info.ranges[inda])) and not(np.isnan(lidar_info.ranges[inda]))):
            a_range = lidar_info.ranges[inda]
        else:
            a_range = 100.0
        
        if(not(np.isinf(lidar_info.ranges[indb])) and not(np.isnan(lidar_info.ranges[indb]))):
            b_range = lidar_info.ranges[indb]
        else:
            b_range = 100.0
        theta = b_angle - a_angle
        alpha=math.atan((a_range*math.cos(theta)-b_range)/(a_range*math.sin(theta)))
        d_t = b_range * math.cos(alpha)

        rospy.loginfo("a_angle = %f", a_angle)
        rospy.loginfo("b_angle = %f", b_angle)
        rospy.loginfo("a_range = %f", a_range)
        rospy.loginfo("b_range = %f", b_range)
        rospy.loginfo("theta= = %f", theta)
        rospy.loginfo("alpha = %f", alpha)
        rospy.loginfo("inda = %f", inda)
        rospy.loginfo("indb = %f", indb)
        rospy.loginfo("d_t = %f", d_t)

        return d_t + 1.00*math.sin(alpha)

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle=0.0

        t_now = rospy.get_rostime().secs
        prev_t=0.0
        dt  = t_now - prev_t
        integral=integral+prev_error
        rospy.loginfo("dt = %f", dt)
        rospy.loginfo("error = %f", error)        
   
        angle=-(kp * error + kd * (error-prev_error)+ ki*integral)

        if(0<=np.abs(angle)<=math.radians(10)):
            velocity=1.5
        elif (10<np.abs(angle)<=math.radians(20)):
            velocity=1.0
        else:
            velocity=0.5
        
        rospy.loginfo("steering_angle= %f",angle)
        rospy.loginfo("velocity %f",velocity)
        prev_error=error
        prev_t=t_now

        drive_result = AckermannDriveStamped()
        drive_result.header.stamp = rospy.Time.now()
        drive_result.header.frame_id = "laser"
        drive_result.drive.steering_angle = angle
        drive_result.drive.speed = velocity
        self.drive_pub.publish(drive_result)


    def followLeft(self, drive_result):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0


    def lidar_callback(self, data):
        """ 
        """
        rospy.loginfo("START //////////////////////////////////////////////////////////")
        d_t1=self.getRange(data)
        rospy.loginfo("d_t1 = %f", d_t1)
        error=DESIRED_DISTANCE_LEFT-d_t1

        self.pid_control(error,VELOCITY)
        
def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    #rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)