#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import PoseStamped, Pose

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('rc_moveit_demo', anonymous=True)
        
 
        # Connect to the arm move group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
       # arm.allow_replanning(True)
        
        arm.set_goal_position_tolerance(0.03)
        arm.set_goal_orientation_tolerance(0.1)
                       
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        
        arm.set_goal_joint_tolerance(0.05)
       
        # Start the arm target in "resting" pose stored in the SRDF file
        arm.set_named_target('up')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)      
        

        arm.set_named_target('down')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)    
             
        arm.set_named_target('left')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)   
        
        arm.set_named_target('right')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)  
        '''
        arm.shift_pose_target(1, -0.05, end_effector_link)
        arm.go()
        rospy.sleep(1)  
        '''
        '''
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.20
        target_pose.pose.position.y = 0.08
        target_pose.pose.position.z = 0.28
        
        
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0
        
        
        
        
        # Set the start state to the current state
        arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        arm.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(5)
        
        '''
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
