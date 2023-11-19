#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "math.h"

class Safety {
private:
    ros::NodeHandle n_;
    double speed;
    
     // TODO: create ROS subscribers and publishers
     
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    nav_msgs::Odometry odometry_info;


public:
    Safety() {
    n_ = ros::NodeHandle();
    speed = 0.0;
      /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message. 
      */
    pub_1 = n_.advertise<std_msgs::Bool>("/brake_bool", 1000);
    pub_2 = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);

    /* You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        
      // TODO: create ROS subscribers and publishers

    */
        sub_1 = n_.subscribe("/scan", 1000, &Safety::scan_callback, this);
        sub_2 = n_.subscribe("/odom", 1000, &Safety::odom_callback, this);
    }
    
    
    void odom_callback (const nav_msgs::Odometry& odometry_msg) {
       // TODO: update current speed
        odometry_info = odometry_msg;
        speed = 0.0;
    }

    void scan_callback(const sensor_msgs::LaserScan& lidar_info) {
    	// TODO: calculate TTC
    	
    	double TTC_threshold = 0.4;
        double min_TTC = 100;
        double v_x = odometry_info.twist.twist.linear.x;
        double v_y = odometry_info.twist.twist.linear.y;
        for (unsigned int i = 0; i < lidar_info.ranges.size(); i++) {
            if (!std::isinf(lidar_info.ranges[i]) && !std::isnan(lidar_info.ranges[i])) {
                double distance = lidar_info.ranges[i];
                double angle = lidar_info.angle_min + lidar_info.angle_increment * i;
                double distance_derivative = cos(angle) * v_x + sin(angle) * v_y;
                if (distance_derivative > 0 && distance / distance_derivative < min_TTC) min_TTC = distance / distance_derivative;
            }
       }
        // TODO: publish drive/brake message
        
        
        if (min_TTC <= TTC_threshold) {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = true;
            pub_1.publish(brake_bool_result);
            ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
            ackermann_drive_result.drive.speed = 0.0;
            pub_2.publish(ackermann_drive_result);
        } else {
            std_msgs::Bool brake_bool_result;
            brake_bool_result.data = false;
            pub_1.publish(brake_bool_result);
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lab2_solution_safety");
    Safety sn;
    ros::Rate loop_rate(100);
    ros::spin();
    return 0;
}
