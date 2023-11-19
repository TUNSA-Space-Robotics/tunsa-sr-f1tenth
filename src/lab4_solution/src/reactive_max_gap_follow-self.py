#!/usr/bin/env python
from __future__ import print_function
from cmath import pi
import sys
import math
from tracemalloc import start
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback,queue_size=100)
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=100)
        self.min_indx = 0
        self.max_indx = 0
        self.start_ = 0
        self.end = 0
        self.best_point_angle = 0.0

    def preprocess_lidar(self, lidar_info):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = lidar_info.ranges
        min_angle = math.radians(-70)
        min_indx = int(math.floor(min_angle-lidar_info.angle_min) / lidar_info.angle_increment)
        max_angle = math.radians(70)
        max_indx = int(math.floor(max_angle-lidar_info.angle_min) / lidar_info.angle_increment)
        for i in range(min_indx,max_indx+1):
            if (np.isinf(proc_ranges[i]) or np.isnan(proc_ranges[i])):
                proc_ranges[i]=0.0
            elif (proc_ranges[i] > lidar_info.range_max):
                proc_ranges[i]=lidar_info.range_max

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        current_start = self.min_indx - 1
        length = 0
        longest_length = 0
        n = 0
        #[1.1 2.3 5.7 0.0 0.0 0.0 0.0 0.0 5.9 5.7 4.2 9.7]
        for i in range(self.min_indx,self.max_indx+1):
            if(current_start < self.min_indx):
                if(free_space_ranges[i] > 0.0):
                    current_start = i
            elif (free_space_ranges[i] == 0.0 and n==0):
                n+=1
                length=i-current_start
                if (length > longest_length):
                    longest_length = length
                    self.start_=current_start
                    self.end=i-1
                    rospy.loginfo("length = %f", length)
            elif (free_space_ranges[i] == 0.0):
                n+=1
            elif(free_space_ranges[i] > 0.0 and n!=0):
                current_start = i
                n=0
        if (free_space_ranges[i-1] > 0.0):                                         ################
            length = self.max_indx + 1 - current_start
            if (length > longest_length):
                longest_length= length
                self.start_=current_start
                self.end=self.max_indx
        rospy.loginfo("longest_length = %f", longest_length)
        tmp2= list(free_space_ranges)                    
        for i in range(self.start_,self.end):
            tmp2[i]=free_space_ranges[i]
        return tuple(tmp2)

    def find_best_point(self,ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        current_max = 0.0
        rospy.loginfo("start_ = {}, end = {}".format(self.start_,self.end))
        for i in range(self.start_,self.end+1):
            if(ranges[i]>current_max):
                current_max=ranges[i]
                self.best_point_angle=ranges.angle_min+i*ranges.angle_increment
            #elif (ranges[i] == current_max):
                #if (np.abs(angle_min+i*angle_increment) < abs(angle)):
                 #   angle=angle_min+i*angle_increment
            #    angle=angle_min+i*angle_increment
            rospy.loginfo("current_max = {} best_point_angle = = {}".format(current_max,self.best_point_angle))
        return self.best_point_angle

    def lidar_callback(self, lidar_info):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(lidar_info)

        #Find closest point to LiDAR (circle center)
        closest_indx = self.min_indx
        closest_distance = lidar_info.range_max
        for i in range(self.min_indx,self.max_indx+1):
            #distance = proc_ranges[i-2] + proc_ranges[i-1] + proc_ranges[i] + proc_ranges[i+1] + proc_ranges[i+1]
            distance=proc_ranges[i]
            if (distance < closest_distance):
                closest_distance = distance
                closest_indx=i

        #Eliminate all points inside 'bubble' (set them to zero) 
        radius = 150
        for i in range(closest_indx-radius,closest_indx+radius+1):
            tmp=list(proc_ranges)
            tmp[i]=0.0
            proc_ranges=tuple(tmp)
        #Find max length gap 
        free_space_ranges = proc_ranges
        self.start_ = self.min_indx
        self.end = self.min_indx
        longest_length_space=self.find_max_gap(free_space_ranges)
        #Find the best point in the gap 
        rospy.loginfo("angle_min = %f", lidar_info.angle_min)
        rospy.loginfo("angle_increment= %d", lidar_info.angle_increment)
        self.best_point_angle=self.find_best_point(longest_length_space)

        #Publish Drive message
        drive_result = AckermannDriveStamped()
        drive_result.drive.steering_angle=self.best_point_angle
        if (np.abs(self.best_point_angle) > 20.0 / 180.0 * math.pi):
            drive_result.drive.speed=0.5
        elif (np.abs(self.best_point_angle) > 10.0 / 180.0 * math.pi):
            drive_result.drive.speed=1.0
        else:
            drive_result.drive.speed = 1.5
        self.drive_pub.publish(drive_result)
        rospy.loginfo("best_point_angle= %f",self.best_point_angle)
def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)