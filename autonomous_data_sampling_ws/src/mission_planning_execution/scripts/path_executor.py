#! /usr/bin/env python3
import rospy
import actionlib
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import JointState
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
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, DisplayTrajectory
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Path
#from service_proxies import *
#from moveit import MoveGroupPythonInterface
#from moveit import MoveGroupCommander
#from moveit import movegroup_interface
#from moveit import planning_scene_interface
#from moveit import robot_commander
#TODO: Add moveit interface

class TrajectoryActionController(object):
    def __init__(self):
        rospy.init_node('rmf_obelix_path_executor')                                                       # init ros node
        rospy.loginfo("Node has started")
        self.pose = Pose()                                                             # init pos for joint states       
        self.path = DisplayTrajectory()                                             
        # Subscriber pose and publish to joint states
        self.sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=self.get_pose)
        #self.commanSub = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callback=self.execute)
        self.pathSub = rospy.Subscriber('/move_group/display_planned_path',DisplayTrajectory,callback=self.get_path)
        self.executeSub = rospy.Subscriber('/execute_rrt_path/chatter',String,callback=self.execute)
        # Publish move command
        self.pub = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=1)        #rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        self.pubCancelAction = rospy.Publisher('/move_base/cancel',GoalID,queue_size=0)
        self.statusPub = rospy.Publisher('/execute_trajectory/status',GoalStatusArray,queue_size=0)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub_rrt = rospy.Publisher("visualization_marker_rrt", Marker, queue_size=10)
        self.marker_pub_rrtstar = rospy.Publisher("visualization_marker_rrtstar", Marker, queue_size=10)
        self.marker_pub_ccr_connect = rospy.Publisher("visualization_marker_rrtconnect", Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/path_topic', Path, queue_size=10)
        self.color_iter = 0
        self.comparepaths = True
        # Load map /move_group/load_map
        # LoadMap = '/map path'
        # map = rospy.ServiceProxy('/move_group/load_map', LoadMap)

        self.xDrone = []
        self.yDrone = []
        self.zDrone = []
        self.pathlist = []
        rospy.loginfo('Successful init')                                                            
        rospy.spin()                                                                                # Loop ros comunication
    def get_path(self, path):
        self.path = path
        rospy.loginfo("New path from planner {}".format(self.path.trajectory[0].multi_dof_joint_trajectory.points))
        self.path = self.path.trajectory[0].multi_dof_joint_trajectory.points   #Store planned waypoints
        self.pathlist.append(self.path)
        print("path list is now {} long".format(len(self.pathlist)))
        #self.matplotPath() # Plot 3d path using matplotlib
        # Visualize planned path in Rviz
        # self.visPath(self.path)
        # Matplot lib visualization
                # Fufill Path message for use in rviz
        
        #draw lines to compare paths from rrt variants
        if self.comparepaths:
            points = Marker()
            line_strip = Marker()
            line_list = Marker()
            points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world"
            points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()
            points.ns = line_strip.ns = line_list.ns = "points_and_lines"
            points.action = line_strip.action = line_list.action = Marker.ADD
            points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0
            points.id = 0
            line_strip.id = 1
            line_list.id = 2
            points.type = Marker.POINTS
            line_strip.type = Marker.LINE_STRIP
            line_list.type = Marker.LINE_LIST
            # POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0
            points.scale.y = 0
            # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_strip.scale.x = 0.1
            line_list.scale.x = 0
            # Points are green
            points.color.g = 1.0
            points.color.a = 1.0
            # Line strip is blue
            line_strip.color.b = 1.0
            line_strip.color.a = 1.0
            # Line list is red
            line_list.color.r = 1.0
            line_list.color.a = 1.0
            #Create the vertices for the points and lines
            for _point in self.path:
                _w =_point.transforms
                _w = _w[0].translation
                p = Point()
                p.x = _w.x
                p.y = _w.y
                p.z = _w.z
                points.points.append(p)
                line_strip.points.append(p)
            # Publish the marker
            #self.marker_pub.publish(points)
            if self.color_iter == 0:
                line_strip.color.r = 1.0
                line_strip.color.a = 1.0
                self.marker_pub_rrt.publish(line_strip)
            elif self.color_iter == 1:
                line_strip.color.g = 1.0
                line_strip.color.a = 1.0
                self.marker_pub_rrtstar.publish(line_strip)
            elif self.color_iter == 2:
                line_strip.color.b = 1.0
                line_strip.color.a = 1.0
                self.marker_pub_ccr_connect.publish(line_strip)

            self.color_iter = self.color_iter +1

    # Callback function for executing trajectory
    def execute(self, goal): 
        rospy.loginfo("Executing callback for trajectory execution beep boop")
        rospy.sleep(1)
        #Init list for storing drone position for plotting
        self.xDrone = []
        self.yDrone = []
        self.zDrone = []
        i= 1
        #Loop though all paths
        for _path in self.pathlist:
            #Loop through all points in trajectory
    
            rospy.loginfo("Executing new path to wpt nr {}".format(i))
            for _point in _path:
                _w =_point.transforms
                #rospy.loginfo("Commanding new waipoint (transforms) {}".format(_w))
                _w = _w[0].translation
                #rospy.loginfo("Commanding new waipoint (.translation){}".format(_w))
                trajectory_execute = MultiDOFJointTrajectory()
                trajectory_execute.header.frame_id = 'world'
                trajectory_execute.header.stamp.secs = rospy.Time.now().secs
                trajectory_execute.header.stamp.nsecs = rospy.Time.now().nsecs
                trajectory_execute.joint_names = []
                # Compose a message containing the waypoint to publish
                quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(0))
                transforms =Transform(translation=Point(_w.x, _w.y, _w.z), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
                velocities = Twist()        # Empty velocity msg
                accelerations = Twist()     # Emptry acceleration msg
                point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
                point.time_from_start.secs = 4
                trajectory_execute.points.append(point)
                self.pub.publish(trajectory_execute)
                #rospy.sleep(0.5)

                #Add drone postion
                self.xDrone.append(self.pose.position.x)
                self.yDrone.append(self.pose.position.y)
                self.zDrone.append(self.pose.position.z)

                #Wait until point is reached
                while not self.tol(_w):
                    rospy.sleep(0.00001)    

                # if point is last point sleep
                if _point == _path[-1]:     
                    rospy.sleep(5)
                    rospy.loginfo("Last point in path, stopping 5s to simulate for data sampling")     
            i = i+1

        # Trajectory is finished, publish status    
        rospy.loginfo("Finished executing mission")
        # Make plots
        self.matplotPath() # Plot 3d path using matplotlib
        self.visPath() # Plot path in Rviz

    def get_pose(self, pose):
        self.pose = pose


    def tol(self, current):
        _pose = self.pose.position
        #Update drone state for plotting
        self.xDrone.append(_pose.x)
        self.yDrone.append(_pose.y)
        self.zDrone.append(_pose.z)
        #Calculate distance to waypoint
        diff_x = (_pose.x - current.x) ** 2
        diff_y = (_pose.y - current.y) ** 2
        diff_z = (_pose.z - current.z) ** 2
        _diff = (diff_x + diff_y + diff_z) ** 0.5
        if _diff <0.2:
            #rospy.loginfo("Diff from waypoint is {}, continuing".format(_diff))
            return True
        else:
            #rospy.loginfo("Diff from waypoint is {}, waiting".format(_diff))
            return False
    def matplotPath(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Extract first pont in path list
        _path = self.pathlist[0]
        _x = []
        _y = []
        _z = []
        for _point in _path:
            _w =_point.transforms
            _w = _w[0].translation
            _x.append(_w.x)
            _y.append(_w.y)
            _z.append(_w.z)
        X = np.array(_x)
        Y = np.array(_y)
        Z = np.array(_z)
        ax.scatter(X[0], Y[0], Z[0], c='green', marker='o', s=100, label='Initial Position')

        label_mark = True
        for _path in self.pathlist:
            _x = []
            _y = []
            _z = []
            for _point in _path:
                _w =_point.transforms
                _w = _w[0].translation
                _x.append(_w.x)
                _y.append(_w.y)
                _z.append(_w.z)
            X = np.array(_x)
            Y = np.array(_y)
            Z = np.array(_z)
            # Plot path only label once
            if label_mark:
                ax.plot(X, Y, Z, label = 'Planned path', ls='solid',color='blue')
                ax.scatter(X[-1], Y[-1], Z[-1], c='red', marker='*', s=100, label='Sample point')
                label_mark = False
            else:
                ax.plot(X, Y, Z, ls='solid',color='blue')
                ax.scatter(X[-1], Y[-1], Z[-1], c='red', marker='*', s=100)
            # Set lables and title
            ax.set_xlabel('X [m]]')
            ax.set_ylabel('Y [m]]')
            ax.set_zlabel('Z [m]]')
            ax.grid(True)
        # Plot drone position if traj executed
        if(self.xDrone  and self.yDrone  and self.zDrone ):
            X_drone = np.array(self.xDrone)
            Y_drone = np.array(self.yDrone)
            Z_drone = np.array(self.zDrone)
            ax.plot(X_drone, Y_drone,Z_drone, label='Quadrotor Position', ls='--', color='black')
            #ax.plot(X_drone, Y_drone, Z_drone, 'ro', label='drone position')
        ax.legend(numpoints=1)    
        plt.show()
        # Plot self.x self.y and self.z in same plot, adjust to same length
        #ax.plot(self.x, self.y, self.z)

        # Make same plot just in 2D
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        ax2.scatter(X[0], Y[0], c='green', marker='o', s=100, label='Initial Position')
        label_mark = True
        for _path in self.pathlist:
            _x = []
            _y = []
            for _point in _path:
                _w =_point.transforms
                _w = _w[0].translation
                _x.append(_w.x)
                _y.append(_w.y)
            X = np.array(_x)
            Y = np.array(_y)
            # Plot path only label once
            if label_mark:
                ax2.plot(X, Y, label = 'Planned path', ls='solid',color='blue')
                ax2.scatter(X[-1], Y[-1], c='red', marker='*', s=100, label='Sample point')
                label_mark = False
            else:
                ax2.plot(X, Y, ls='solid',color='blue')
                ax2.scatter(X[-1], Y[-1], c='red', marker='*', s=100)
            # Set lables and title
            ax2.set_xlabel('X [m]]')
            ax2.set_ylabel('Y [m]]')
            ax2.grid(True)
        # Plot drone position if traj executed
        if(self.xDrone  and self.yDrone  and self.zDrone ):
            X_drone = np.array(self.xDrone)
            Y_drone = np.array(self.yDrone)
            ax2.plot(X_drone, Y_drone, label='Quadrotor Position', ls='--', color='black')
            #ax.plot(X_drone, Y_drone, Z_drone, 'ro', label='drone position')
        ax2.legend(numpoints=1)    
        plt.show()



    def visPath(self):
        # Fufill Path message for use in rviz
        points = Marker()
        line_strip = Marker()
        line_list = Marker()
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world"
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()
        points.ns = line_strip.ns = line_list.ns = "points_and_lines"
        points.action = line_strip.action = line_list.action = Marker.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0
        points.id = 0
        line_strip.id = 1
        line_list.id = 2
        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP
        line_list.type = Marker.LINE_LIST
        # POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0
        points.scale.y = 0
        # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1
        line_list.scale.x = 0
        # Points are green
        points.color.g = 1.0
        points.color.a = 1.0
        # Line strip is blue
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0
        # Line list is red
        line_list.color.r = 1.0
        line_list.color.a = 1.0
        # Create the vertices for the points and lines
        #for _path in self.pathlist:
        #    for _point in _path:
        #        _w =_point.transforms
        #        _w = _w[0].translation
        #        p = Point()
        #        p.x = _w.x
        #        p.y = _w.y
        #        p.z = _w.z
        #        points.points.append(p)
        #        line_strip.points.append(p)
        ## Publish the marker
        ##self.marker_pub.publish(points)
        #self.marker_pub.publish(line_strip)
        #self.marker_pub.publish(line_list)
        rospy.sleep(.1)
        
        # make line for actual drone position
        if(self.xDrone  and self.yDrone  and self.zDrone ):
            X_drone = np.array(self.xDrone)
            Y_drone = np.array(self.yDrone)
            Z_drone = np.array(self.zDrone)
            line_strip_drone = Marker()
            line_strip_drone.header.frame_id = "world"
            line_strip_drone.header.stamp = rospy.Time.now()
            line_strip_drone.ns = "points_and_lines"
            line_strip_drone.action = Marker.ADD
            line_strip_drone.pose.orientation.w = 1.0
            line_strip_drone.id = 3
            line_strip_drone.type = Marker.LINE_STRIP
            line_strip_drone.scale.x = 0.1
            # Line strip is red
            line_strip_drone.color.r = 1.0
            line_strip_drone.color.a = 1.0
            for i in range(len(X_drone)):
                p = Point()
                p.x = X_drone[i]
                p.y = Y_drone[i]
                p.z = Z_drone[i]
                line_strip_drone.points.append(p)
            self.marker_pub.publish(line_strip_drone)
            rospy.sleep(.1)





if __name__ == '__main__':
    TrajectoryActionController()                                     # Run class constructor on program start 