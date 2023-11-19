# TUNSA Space Robotics F1TENTH Workspace

This ROS workspace contains our proposed solutions for the first 5 [labs](https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/index.html) of the [F1TENTH Autonomous Vehicle Course](https://f1tenth-coursekit.readthedocs.io/en/stable/introduction/overview.html) provided by the University of Pennsylvania. During our work, we followed this [plan](https://docs.google.com/spreadsheets/d/12VkkMe5WgANmVBByFx13zXS2Ez5oGKWnXdzBt8jrHEU/edit?usp=sharing). For each lab, we were provided with a template repository, which contains the latex source files for lab instructions as well as any skeleton code.

Lab 1 - Introduction to ROS
=============================

**Goals:** 
The goal of this lab assignment is to get you familiar with the various paradigms and uses of ROS and how it can be used to build robust robotic systems. ROS is a meta-operating system that simplifies inter-process communication between elements of a robot's perception planning and control systems.

**Learning Outcomes:** 
Upon completion of this lab, we were able to understand the following fundamentals:

	* Understanding the directory structure and framework of ROS
	* Understanding how publishers and subscribers are implemented
	* Implementing custom messages
	* Understanding Cmake lists and package.XML files
	* Understanding dependencies
	* Working with launch files
	* Working with Rviz
	* Working with Bag files
 
**Required Skills:** Python or C++ (or at least some programming experience)

[**Lab Template Repository**](https://github.com/f1tenth/f1tenth_labs/tree/master/lab1/latex>)

Lab 2 - Automatic Emergency Braking
======================================

**Goals:**
| The goal of this lab is to develop a safety node for the race cars that will stop the car from collision when travelling at higher velocities. We will implement Time to Collision using the LaserScan message in the simulator. 

**Learning Outcomes:**
Upon completion of this lab, we were able to understand the following fundamentals:

	* Using the LaserScan message in ROS
	* Time to Collision (TTC)
	* Safety critical systems

**Required Skills:** Basics of ROS from Lab 1, Python or C++ (or at least some programming experience)

[**Lab Template Repository**](https://github.com/f1tenth/f1tenth_labs/tree/master/lab2>)

Lab 3 - Wall Following
=======================

**Goals:**
In this lab, you will implement a PID (proportional integral derivative) controller to make the car drive parallel to the walls of a corridor at a fixed distance. At a high level, you will accomplish this by taking laser scan distances from the Hokuyo LiDAR, computing the required steering angle and speed (drive parameters), an publishing these to the VESC to drive the car. 

**Learning Outcomes:**
Upon completion of this lab, we were able to understand the following fundamentals:

	* PID controllers
	* Driving the car autonomously via Wall Following

**Required Skills:** ROS, Python/C++

[**Lab Template Repository**](https://github.com/f1tenth/f1tenth_labs/tree/master/lab3>)

Lab 4 - Follow the Gap
===========================================

**Goals:**
In this lab, you will implement a reactive algorithm for obstacle avoidance. While the base starter code defines an implementation of the F1TENTH Follow the Gap Algorithm, you are allowed to submit in C++, and encouraged to try different reactive algorithms or a combination of several. In total, the python code for the algorithm is only about 120 lines.

**Learning Outcomes:**
Upon completion of this lab, we were able to understand the following fundamentals:

	* Reactive methods for obstacle avoidance

**Allotted Time:** 1 week

[**Lab Template Repository**](https://github.com/f1tenth/f1tenth_labs/tree/master/lab4>) 

Lab 5 - Scan Matching
======================

**Goals:**
This lab deals with the problem of localization in Robotics and provides an introduction to localization and why is it important in the autonomy stack. Through the lab, one of the most fundamental algorithms of localization, *scan matching*, is implemented. It uses the *Iterative Closest Point* algorithm which has been introduced in class. You can take reference from the [Andre Censi PLICP paper](https://censi.science/pub/research/2008-icra-plicp.pdf). By the end of this lab you will have a certain level of knowledge and expertise in localization of a robot given a mapped environment and how it is important in path planning and trajectory tracking.

**Learning Outcomes:**
Upon completion of this lab, we were able to understand the following fundamentals:

	* Localization
	* Odometry Estimation
	* Convex optimization
	* C++ OOP
	* Quadratic Programming

**Required Skills:** ROS, Python/C++

[**Lab Template Repository**](https://github.com/f1tenth/f1tenth_labs/tree/master/lab5)
