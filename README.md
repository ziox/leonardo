# Project Leonardo

**Marker based localization and navigation with a Parrot AR Drone 2.0** 

<img src=http://www.volarecon.it/ridimensiona-dicodici.jpg width="850" title="Macchina volante di Leonardo">

This project was developed for the Robotics course of the "Università degli Studi di Bergamo", taught by Prof. Davide Brugali.

Authors

 - Nicola Pasta
 - Roberto Gazia
 - Giuliano Breviario
 - Claudio Capelli


## Introduction
Leonardo is a collection of ROS nodes that allow a quadcopter to fly autonomously.
It uses the [ardrone_autonomy] driver and the [ArUco] library to localize and fly a Parrot AR Drone.

We use the ArUco library to recognize markers placed on the ground as seen by the Parrot bottom camera.
When a marker is visible, we can estimate the drone's location based on our knowledge of the marker's position.
After we've estimate our position, we use that information to navigate the drone through the environment.


## Installation
 
To use this project you'll need a working copy of ROS. ROS is a flexible framework for writing robot software.
It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms
 
This project use ROS Indigo Igloo, but any newer release should work.
 
 
### Linux
 
ROS Indigo is compatible with Ubuntu Saucy (13.10) and Trusty (14.04). We chose to install a lightweight version of Ubuntu Trusty, Lubuntu 14.04.
Lubuntu uses the minimal desktop LXDE, and a selection of light applications.
It is focused on speed and energy-efficiency and because of this, it has very low hardware requirements that make it suitable for the use in a virtual machine.
 
 
### ROS
 
There's a lot of documentation on how to install ROS, here's an overview of the process.
 
* Preparation

    `$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list`

    `$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -`

    `$ sudo apt-get update`
 
* Packages:

    We installed a bare bone version of ROS and we added only the packages that we needed
	
    `$ sudo apt-get install ros-indigo-ros-base`

    `$ sudo apt-get install ros-indigo-cv-bridge`
    
    `$ sudo apt-get install ros-indigo-rviz`

    `$ sudo apt-get install ros-indigo-ardrone-autonomy`
 
* Initialize rosdep 

    `$ sudo rosdep init`

    `$ rosdep update`
 
You may need to setup your environment accordingly
 
 
### OpenCV
 
OpenCV (Open Source Computer Vision Library) is an open source computer vision and machine learning software library.
 
Since ROS Electric, the OpenCV framework is no longer a ROS core package.
This library is needed for our marker detection node, so make sure it's installed in your system (it should be installed as a dependency for `ros-indigo-cv-bridge`).
 
This project uses OpenCV version 2.4.8.
 
 
### ArUco
 
ArUco is a minimal C++ library for Augmented Reality applications developed at the University of Cordoba and based on OpenCV.
 
* Download and extract the ArUco library version 1.2.5

    `$ wget http://sourceforge.net/projects/aruco/files/1.2.5/aruco-1.2.5.tgz`

    `$ tar zxvf aruco-1.2.5.tgz`
	
* Build it!

    `$ cd aruco-1.2.5`

    `$ mkdir build`

    `$ cd build`
    
    `$ cmake ..`

    `$ sudo make install`
 
 
 # More ...
