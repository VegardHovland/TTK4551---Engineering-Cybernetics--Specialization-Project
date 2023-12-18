#! /usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
import rospy
import actionlib
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import JointState, MultiDOFJointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray,Header
import time
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
#from trajectory_msgs.msg import (JointTrajectoryPoint)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist
from actionlib_msgs.msg import GoalStatusArray, GoalID, GoalStatus
import math
import tf
#from joint_command_interface.action import FollowJointTrajectoryAction, MultiDOFJointTrajectoryAction 
#from joint_command_interface.msg import      
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, DisplayTrajectory, PlanningScene
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import numpy as np
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import sys

class QuadrotorCommander:
    def __init__(self):
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "rmf_obelix_planning_group"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        self.pose = Pose()
        rospy.sleep(5)                                                            # init pos for joint states       
        self.sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=self.get_pose)
        #/move_group/monitored_planning_scene
        self.plansub = rospy.Subscriber("move_group/monitored_planning_scene", PlanningScene, callback=self.get_planning_scene)
        self.planscene_init  = 0
    def set_start_position(self):
        # Set the starting pose
        start_pose = PoseStamped()
        start_pose.header.frame_id = "world"
        start_pose.pose.position.x = self.pose.position.x
        start_pose.pose.position.y = self.pose.position.y
        start_pose.pose.position.z = self.pose.position.z
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(start_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_waypoint(self, x, y, z):
        # setstart state
        if self.planscene_init == 0:
            return
        #print planining scene
        #print(self.scene)
        self.move_group.set_start_state_to_current_state()
        # Set the target pose
        pose_target = Pose()

        #pose_target.header.frame_id = "world"
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.w = 1.0
        self.move_group.set_pose_target(pose_target)

        #self.move_group.set_start_state()
        # Plan 
        self.move_group.set_planning_time(10)
        self.move_group.set_workspace([-x, -y, -z, x, y, z])
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_num_planning_attempts(10)
        plan = self.move_group.go(wait=True)
        #self.move_group.stop()
        #self.move_group.clear_pose_targets()
        
        #Dsiplay trajectory
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        print(display_trajectory)
        try:
            self.display_trajectory_publisher.publish(display_trajectory)
        except:
            print("Error calculating trajectory")
            pass

    def get_pose(self, pose):
        # Get pose
        self.pose = pose
        #print("Pose: {}".format(self.pose))
    def get_planning_scene(self, scene):
        self.planscene_init = 1
        print("planning scene")
        #print("Planning scene: {}".format(scene))
        self.scene = scene
    def debugMovegroup(self):
        # We can get the name of the reference frame for this robot:
        #planning_frame = self.group.get_planning_frame()
        #print("============ Reference frame: %s" % planning_frame)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:", group_names)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    
    

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('quadrotor_commander', anonymous=True)
        # Create the MoveIt commander interface
        quadrotor_commander = QuadrotorCommander()
        # set planning time to 10s and workspace to 10x10x5m
        for i in range(5):
            quadrotor_commander.move_group.set_planning_time(10)
            quadrotor_commander.move_group.set_workspace([-5, -5, -5, 5, 5, 5])
            quadrotor_commander.move_group.set_planner_id("RRTConnectkConfigDefault")
            quadrotor_commander.move_group.set_num_planning_attempts(10)
            quadrotor_commander.move_group.set_max_velocity_scaling_factor(1.0)
            quadrotor_commander.move_group.set_max_acceleration_scaling_factor(1.0)
            quadrotor_commander.move_group.set_goal_tolerance(0.1)
            rospy.sleep(1)
        quadrotor_commander.move_group.set_start_state_to_current_state()
        rospy.sleep(2)
        # Set waypoint list of points to visit all rooms in floor 2 of the building
        waypoint_list = [[-6.635 * 10^-5,	9.45*10^-5,	-7.79*10^-5],
                          [4.03 *10^-5, 9.9 * 10^-5,	9.5*10^-5],
                          [3.0245186300290747, -2.5910522184447387, 0.8949843896790408],
                          [3.966754822403286, 1.7774597521147226, 0.8269618500032928],
                          [-1.8788720537224786, 2.5702150996482, 1.1510636716570706]]
        # Set the starting position of the quadrotor
        while True:
            for wpt in waypoint_list:
                quadrotor_commander.go_to_waypoint(wpt[0], wpt[1], wpt[2])
                rospy.sleep(2)


    except rospy.ROSInterruptException:
        pass
