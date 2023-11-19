#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
# kp , kd , ki are chosen arbitrarily
kp = 1.0
kd = 0.001
ki = 0.005
"""
 Vo = Kp * e(t) + Ki * E(t) + kd * d e(t)/dt 
          |        |        |
          P        I        D
 Vo = Kp * error + Ki * integral  + Kd * prev_error - current_error
e(t) == projected error 
E(t) == accumulated error by delta time
if Ki is incresed == over-correction ==> may result in crash 
steering angle = cureent_angle - Vo (correction)
"""
servo_offset = 0.0
prev_error = 0.0 
error = 0.0 # current error 
integral = 0.0
prev_time = 0.0
#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.85
VELOCITY = 1.5 # meters per second
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        global prev_time 
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        #TODO: Subscribe to LIDAR
        self.lidar_sub =  rospy.Subscriber(lidarscan_topic , LaserScan ,self.lidar_callback , queue_size=1000)
        #TODO: Publish to drive
        self.drive_pub = rospy.Publisher(drive_topic , AckermannDriveStamped ,queue_size=1000)

        prev_time = rospy.get_time()
        
    def getRange(self, data, angle): # filter scan reading
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        """
        angle: between -45 to 225 degrees (270)
        """
        if -45 <= angle <= 225 :
            idx = len(data) * ( angle + 90 ) / 360
            ray = data[int(idx)]
            if not np.isnan(ray) and not np.isinf(ray):
                return ray 
        


    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        #TODO: Use kp, ki & kd to implement a PID controller for 

        current_time =  rospy.get_time()
        delta = current_time - prev_time
        integral += prev_error * delta # I
        angle = -( kp * error +  ki * integral + kd * (error - prev_error) / delta ) 
        prev_error = error
        prev_time = current_time


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now() # current time
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        

        # if   0 < steering angle < 20 ==> speed = 1.5 
        if abs(angle) <= math.radians(10):
            drive_msg.drive.speed = velocity
        # if 10 < steering angle < 20 ==> speed = 1.0
        elif math.radians(10) <= abs(angle) <= math.radians(20) :
            drive_msg.drive.speed = 1.0
        else :
            drive_msg.drive.speed = 0.5
        # publish    
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        b_angle = 125 
        a_angle = 180
        theta = math.radians(abs(b_angle - a_angle))
        b = self.getRange(data , b_angle) 
        a = self.getRange(data , a_angle) # x axis 

        alpha_angle = math.atan(( b * math.cos(theta) - a ) / (b * math.sin(theta) ))
        AB_dist = a * math.cos(alpha_angle) # future distance

        CD_dist = AB_dist + CAR_LENGTH * math.sin(alpha_angle)


        return leftDist - CD_dist

    def lidar_callback(self, data):
        """
        error = desired - actual  
        """
        

        error =  self.followLeft(data.ranges , DESIRED_DISTANCE_LEFT)
        #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
