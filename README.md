# Leonardo

## Introduction

Here are overviews of the operating system, software and concepts used in the project.

### Operating System - Linux

Linux is a Unix-like and POSIX compliant computer operating system, originally developed as a free and open-source software for Intel x86-based personal computers.
The defining component of Linux is the Linux kernel, released on 5 October 1991 by Linus Torvalds.
Linux is packaged in a form known as a Linux distribution. Some popular Linux distributions include Debian, Ubuntu, Linux Mint, Fedora, openSUSE, Arch Linux. Linux distributions include the Linux kernel, supporting utilities and libraries and usually a large amount of application software to fulfill the distribution's intended use.

#### Lubuntu

In this project we used the distribution known as Lubuntu. It's a fast and lightweight operating system. The core of the system is based on Linux and Ubuntu. Lubuntu uses the minimal desktop LXDE, and a selection of light applications. We focus on speed and energy-efficiency. Because of this, Lubuntu has very low hardware requirements that make it suitable for the use on a virtual machine like in this project.

### ROS - Robot Operating System

ROS is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

#### Core components

- **Roscore**: Collection of nodes and programs that are pre-requisites of a ROS-based system. Must have a roscore running in order for ROS nodes to communicate.

- **Nodes**: A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. A robot control system will usually comprise many nodes.

- **Comunication protocol - Publish/Subscribe**: The publisher writes a message on a topic provided by the roscore. All nodes that want to receive the message can request it to roscore and they are known as subscribers.

### Marker based navigation

The core problem of the project is the navigation of an ARDrone based on marker. In the marker based navigation the robot moves between two positions by following a
topological path consisting of a sequence of visual markers.
The details about the implementation are in the Marker detection chapter.

<div style="text-align:center">
<img src="https://dl.dropboxusercontent.com/u/52788948/Marker%20navigation.PNG" ></div>

### Filtering

- Filtraggio
- eventuale virtuale machine

## AR Drone

### Introduction

### Camera

### Wi-fi connection

## Marker detection

One of the main problem in marker based navigation is to identify a marker using the camera. Here are the library used to solve this issue.

### ARToolkit

ARToolKit is a software library for building Augmented Reality applications. These are applications that involve the overlay of virtual imagery on the real world. We used this library only at the beginning of the project, but...

### Aruco

Working on the project we found a better solution to the marker detection provided by the Aruco library. 
Aruco is a minimal C++ library for Augmented Reality applications based on OpenCv developed by the University of Cordoba.

We also made a modify on the library deleting the code used to print on the console.

<div style="text-align:center">
<img src="https://www.google.it/url?sa=i&rct=j&q=&esrc=s&source=images&cd=&cad=rja&uact=8&ved=0CAcQjRw&url=http%3A%2F%2Fiplimage.com%2Fblog%2Fcv-img-tec-black-white-marker-detection%2F&ei=KAgtVeqdCsnEPeKNgcgH&bvm=bv.90790515,d.d2s&psig=AFQjCNG3uXibJyAKzKjlM0KeC0Uw7cawsQ&ust=1429100937120769" ></div>

The library relies on the use of coded markers. Each marker has an unique code indicated by the black and white colors in it. The libary detect borders, and analyzes into the rectangular regions which of them are likely to be markers. Then, a decoding is performed and if the code is valid, it is considered that the rectangle is a marker.

The codification included into the marker is a slighly modified version of the Hamming Code. It has a total of 25 bits didived in 5 rows of 5 bits each. So, we have 5 words of 5 bits. Each word, contains only 2 bits of real information, the rest is for  and error detection/correction (error correction is yet to be done). As a conclusion, a marker contains 10 bits of real information wich allows 1024 different markers.

<div style="text-align:center">
<img src="https://www.google.it/url?sa=i&rct=j&q=&esrc=s&source=images&cd=&cad=rja&uact=8&ved=0CAcQjRw&url=https%3A%2F%2Fgithub.com%2Fjchillerup%2Frokoko%2Fblob%2Fmaster%2Freadme.md&ei=QAgtVYKmOMrXPZqGgMgJ&bvm=bv.90790515,d.d2s&psig=AFQjCNG3uXibJyAKzKjlM0KeC0Uw7cawsQ&ust=1429100937120769" ></div>


#### Aruco classes

The ArUco library contents are divided in two main directories. The src directory, which contains the library itself. And the utils directory which contains the applications.

The library main classes are:

   - **aruco::Marker**: which represent a marker detected in the image;
   - **aruco::MarkerDetector**: that is in charge of deteting the markers in a image Detection is done by simple calling the member funcion ArMarkerDetector::detect(). Additionally, the classes contain members to create the required matrices for rendering using OpenGL. See aruco_test_gl for details;
   - **aruco::BoardConfiguration**: A board is an array of markers in a known order. BoardConfiguracion is the class that defines a board by indicating the id of its markers. In addition, it has informacion about the distance between the markers so that extrinsica camera computations can be done;
   - **aruco::Board**: This class defines a board detected in a image. The board has the extrinsic camera parameters as public atributes. In addition, it has a method that allows obtain the matrix for getting its position in OpenGL;
   - **aruco::BoardDetector**: This is the class in charge of detecting a board in a image. You must pass to it the set of markers detected by ArMarkerDetector and the BoardConfiguracion of the board you want to detect. This class will do the rest for you, even calculating the camera extrinsics.



   
- Calib.

## Architettura

- UML? (Poco)
- Schema a blocchi

## Progetto
(descrizione di quello che boh?)

- Teleop
- Aruco Marker Detector
- Odometry
- Localization
- Controller

## Conclusioni

conclusione
