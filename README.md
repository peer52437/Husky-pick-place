# Husky-pick-place
This is the repository for the project Husky Pick and Place, made specifically for the minor Adaptive Robotics (AR) to gain levels in Principles of Robotics (POR) and the Robot Operating System (ROS) classes on Fontys University of Applied Sciences in Eindhoven, the Netherlands. This project is experimental in nature and further updates to the system will not be made.

## Goals:
The goal of this project was to demonstrate the knowledge the project members gained during the minor in POR and ROS. 
The members of this project have done this using a combination of a Husky Robot, by [Clearpath Robotics](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) and an UR5 robotic arm, by [Universal Robotics](https://www.universal-robots.com/).
The goal is for the Husky to drive to point A, pick up a object, drive to point B and place the object. This will be done in a simulated environment. The manipulator used is the UR5 and a SICK laser scanner is ued for navigation.
These two accesories can be turned on and off in the launch file por_lvl3. Unfortunately the URDF is originally not built to enable the user to use two different accesories at the same time. This meant that the UR5 and the laser scanner were inside of one another.
To enable the husky to use the UR5 and the laser scanner at the same time, the urdf file is edited.

## The Process
The process is described in our [wiki](https://github.com/peer52437/Husky-pick-place/wiki), with a step-by-step guide to achieving the same result.

## The Simulation
The pick and place project was simulated in gazebo and RViz. In the simulation the Husky picks up a object from a dumpster and then places that object on another dumpster.
A video of the simulation can be found here:
https://youtu.be/P1JF098UJJg
Every time a command is given, the coordinates of the next waypoint is sent to the move_base node. This triggers the path planning (Dijkstra's algorithm) to calculate a path towards it.
When the waypoint has been reached, the next python script is triggered, which sends the angles of each joint to the moveit controller so it can calculate the path towards that point.

An additional simulation was made were the environment. In this simulation a box was placed in the path of the Husky. The Husky tried to find a path but was unable to. so changing the environment (without saving the changes in the map) is not possible.
https://youtu.be/ecWTlAxEkDU
