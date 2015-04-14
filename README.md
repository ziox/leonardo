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
<img src="https://dl-web.dropbox.com/get/Public/Marker%20navigation.PNG?_subject_uid=52788948&w=AABNJQUbnwxnGEGfMs5ThupfyjZ-e0dsrpQl4DSvdTeWKw">


- Filtraggio
- eventuale virtuale machine

## AR Drone

## Marker detection

- ARToolkit (faceva cagare)
- Aruco (fa meno cagare)
  (fix lib logs)
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
