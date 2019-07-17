# Augmented Reality Framework in Unity for ROS

## Introduction

The Unity code for the device end can be found [here](https://github.com/faizan-m/arfuros). This repository contains the ROS package that needs to be installed on the robot end.

The goal for this framework is to allow robots to express their internal state (sensory, cognition, planning) as virtual objects over the real world for human users to see. The long term goal for this framework is to make it adaptive so that it corresponds more and more to the robot's situational awareness which it can exploit to better express itself in AR.

Currently, all the code is customized to work with Turtlebot robots and there are several dependencies and assumptions made in the framework to work with our lab's software architecture for ROS robots. Therefore, it is not currently in a publicly usable state though as we work through with it, it will become much better.

## How to Use
This repository contains the package that needs to be installed on the robot. [This](https://github.com/faizan-m/arfuros) repository contains code that can be compiled for the user devices. 

## How to Run
For most uses of the arfuros project, and if the Turtlebot has a lidar, first run `roscore`, then in a separate terminal window run `roslaunch arfuros tbot2_lidar.launch`. If the Turtlebot does not have a lidar, instead run `roslaunch arfuros tbot2.launch`.
