#!/usr/bin/env python
#This code was written for POR/ROS lvl 3 for the minor Adaptive Robotics by Fontys University of Applied Sciences in Eindhoven, The Netherlands.
#This code will start a menu where the operator can input a specific location or make the robot loop through the locations.
#These locations are changed using the parameters in this code.
#When the robot has arrived at the location, the UR5 arm will be extended to simulate a pickup. The angles of the joints can also be changed in the pos1_ur5.py file.

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
import os



class map_navigation():
    def choose(self):

        choice = 'q'

        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESS A KEY:")
        rospy.loginfo("|'0': Start ")
        rospy.loginfo("|'1': Waypoint 1 ")
        rospy.loginfo("|'2': Waypoint 2 ")
        rospy.loginfo("|'10': Rondje ")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice

    def __init__(self):


        # declare the coordinates of interest
        self.xStart = 0.028
        self.yStart = 0.024
        self.zStart = 0.001
        self.wStart = 1.000

        self.xWaypoint_1 = -1.466
        self.yWaypoint_1 = -3.253
        self.zWaypoint_1 = -0.489
        self.wWaypoint_1 = 0.872

        self.xWaypoint_2 = 1.975
        self.yWaypoint_2 = 7.534
        self.zWaypoint_2 = 0.984
        self.wWaypoint_2 = 0.175


        self.goalReached = False
        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)

	rospy.sleep(10)							
        choice = self.choose()

        if (choice == 0):

            	self.goalReached = self.moveToGoal(self.xStart, self.yStart, self.zStart, self.wStart)
		if (self.goalReached):
                	rospy.loginfo("Stowing ur5")
			os.system('rosrun por stow_ur5')


        elif (choice == 1):

	        self.goalReached = self.moveToGoal(self.xWaypoint_1, self.yWaypoint_1, self.zWaypoint_1, self.wWaypoint_1)
		if (self.goalReached):
                	rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
			rospy.loginfo("Stowing ur5")



        elif (choice == 2):

            	self.goalReached = self.moveToGoal(self.xWaypoint_2, self.yWaypoint_2, self.zWaypoint_2, self.wWaypoint_2)
		if (self.goalReached):
			rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
                	rospy.loginfo("Stowing ur5")


        elif (choice == 10):

            self.goalReached = self.moveToGoal(self.xStart, self.yStart, self.zStart, self.wStart)

            if (self.goalReached):
                rospy.loginfo("Stowing ur5")
		os.system('rosrun husky_control stow_ur5')
                # rospy.spin()


            # rospy.spin()

            else:
                rospy.loginfo("Hard Luck!")

            import time
            time.sleep(5)

            self.goalReached = self.moveToGoal(self.xWaypoint_1, self.yWaypoint_1, self.zWaypoint_1, self.wWaypoint_1)

            if (self.goalReached):
		rospy.loginfo("Grabbing stuff")
		os.system('rosrun por moveit.py')
                rospy.loginfo("Stowing ur5")
                # rospy.spin()


            # rospy.spin()

            else:
                rospy.loginfo("Hard Luck!")

            import time
            time.sleep(5)

            self.goalReached = self.moveToGoal(self.xWaypoint_2, self.yWaypoint_2, self.zWaypoint_2, self.wWaypoint_2)

            if (self.goalReached):
		rospy.loginfo("Grabbing stuff")
		os.system('rosrun por moveit.py')
                rospy.loginfo("Stowing ur5")

                # rospy.spin()


            # rospy.spin()

            else:
                rospy.loginfo("Hard Luck!")

            
        while choice != 'q':
            choice = self.choose()
            if (choice == 0):

                self.goalReached = self.moveToGoal(self.xStart, self.yStart, self.zStart, self.wStart)
		if (self.goalReached):
                	rospy.loginfo("Stowing ur5")
			os.system('rosrun husky_control stow_ur5')

            elif (choice == 1):

                self.goalReached = self.moveToGoal(self.xWaypoint_1, self.yWaypoint_1, self.zWaypoint_1,
                                                   self.wWaypoint_1)
		if (self.goalReached):
                	rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
			rospy.loginfo("Stowing ur5")


            elif (choice == 2):

                self.goalReached = self.moveToGoal(self.xWaypoint_2, self.yWaypoint_2, self.zWaypoint_2,
                                                   self.wWaypoint_2)
		if (self.goalReached):
			rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
                	rospy.loginfo("Stowing ur5")

												   
            elif (choice == 10):

                self.goalReached = self.moveToGoal(self.xStart, self.yStart, self.zStart, self.wStart)

                if (self.goalReached):
                	rospy.loginfo("Stowing ur5")
			os.system('rosrun husky_control stow_ur5')
                    # rospy.spin()


                # rospy.spin()

                else:
                    rospy.loginfo("Hard Luck!")

                import time
                time.sleep(5)

                self.goalReached = self.moveToGoal(self.xWaypoint_1, self.yWaypoint_1, self.zWaypoint_1,
                                                   self.wWaypoint_1)

                if (self.goalReached):
			rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
                 	rospy.loginfo("Stowing ur5")

                    # rospy.spin()


                # rospy.spin()

                else:
                    rospy.loginfo("Hard Luck!")

                import time
                time.sleep(5)

                self.goalReached = self.moveToGoal(self.xWaypoint_2, self.yWaypoint_2, self.zWaypoint_2,
                                                   self.wWaypoint_2)

                if (self.goalReached):
			rospy.loginfo("Grabbing stuff")
			os.system('rosrun por moveit.py')
                    	rospy.loginfo("Stowing ur5")

                    # rospy.spin()

                    
                # rospy.spin()

                else:
                    rospy.loginfo("Hard Luck!")
                    
                

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

    def moveToGoal(self, xGoal, yGoal, zGoal, wGoal):

        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = zGoal
        goal.target_pose.pose.orientation.w = wGoal

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if (ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False


if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
