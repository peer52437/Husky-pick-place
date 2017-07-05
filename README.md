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

## Updating Simulated Environment
An additional simulation was made where the environment was changed. In this simulation a box was placed in the path of the Husky. The Husky tried to find a path but was unable to. so changing the environment (without saving the changes in the map) is not possible.
To be able to have a working simulation with changing environment the navigation xml has to be changed to incorporate the current sensor data so the path planning algorithm can be updated.
https://youtu.be/ecWTlAxEkDU

## MoveIt problems
In the current simulation, MoveIt is not used to plan the movements of the UR5 arm. We have incorporated moveit into the RViz environment and can use the interactive markers to plan movements in RViz and execute them, also visualizing this in Gazebo.
We have tried using a script to have the UR5 arm move to coordinates, but the kinematic solver of MoveIt would not instatiate, giving this error message:

    Kin chain provided in model doesn't contain standard UR joint 'shoulder_lift_joint'.
    Kinematics solver of type 'ur_kinematics/UR5KinematicsPlugin' could not be initialized for group 'ur5_arm'
    Kinematics solver could not be instantiated for joint group ur5_arm.
    Ready to take MoveGroup commands for group ur5_arm.
    Replanning: yes
    Fail: ABORTED: No motion plan found. No execution attempted.
This meant that the kinematic Solver didn't work for an unknown reason. After some searching on the web, we found that a different kinematic solver had to be used for the UR5, it's called TRAC_IKKinematicsPlugin.
By changing the Kinematics.yaml file to incorporate the new kinematic solver, we solved the errors above.
The next error we encountered was that the solver could not find a path. This is caused by MoveIt which tries to find a path within a certain time. If it can't plot a path in this time it will give this error. To solve this error, smaller steps have to be taken to stay within the timeslot. 
After solving this a frame was also placed in RViz with a transformation from the base_link of the husky. This frame represents a possible object to pick and place.
A video with moveit and the object frame can be found here:
https://youtu.be/sDt0YGg1weY

This program with moveit was then used in the program of VeelWaypoints.py. So in this simulation the robot drives to the waypoints and simulates picking and placing an object. A video was also made of this simulation.
https://youtu.be/95B99MdWNBs

For a better simulation an object would have to be placed in gazebo. Then that object needs to get a frame in RViz.
Then when the husky drives to this object it has to make a tf to the robot and send the arm to the location of the frame.
This has not succeeded so far for two reasons. first, reading the quaternions and sending the arm there has proved difficult. And second creating a frame in RViz from an object in gazebo.
