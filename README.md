# Husky-pick-place
This is the repository for the project Husky Pick and Place, made specifically for the minor Adaptive Robotics (AR) to gain levels in Principles of Robotics (POR) and the Robot Operating System (ROS) classes on Fontys University of Applied Sciences in Eindhoven, the Netherlands.

## Goals:
The goal of this project was to demonstrate the knowledge the project members gained during the minor in POR and ROS. 
The members of this project have done this using a combination of a Husky Robot, by [Clearpath Robotics](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) and an UR5 robotic arm, by [Universal Robotics](https://www.universal-robots.com/).
The Husky is placed in a simulated environment with an UR5 as manipulator, and SICK laser scanner for navigation. These two accesories can be turned on and off in the launch file por_lvl3. Unfortunately the URDF is originally not built to enable the user to use two different accesories at the same time. This meant that the UR5 and the laser scanner were inside of one another.
To enable the husky to use the UR5 and the laser scanner at the same time, the urdf file is edited.

## The Process
The process is described in our [wiki](https://github.com/peer52437/Husky-pick-place/wiki), with a step-by-step guide to achieving the same result.
