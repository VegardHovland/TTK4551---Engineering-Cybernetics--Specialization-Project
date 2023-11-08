#! /usr/bin/env python3
import rospy
import actionlib
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
#from service_proxies import *
#from moveit import MoveGroupPythonInterface
#from moveit import MoveGroupCommander
#from moveit import movegroup_interface
#from moveit import planning_scene_interface
#from moveit import robot_commander
#TODO: Add moveit interface

class TrajectoryActionController(object):
    def __init__(self):
        rospy.init_node('rmf_obelix_plan_interface')                                                       # init ros node
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
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        # Load map /move_group/load_map
        # LoadMap = '/map path'
        # map = rospy.ServiceProxy('/move_group/load_map', LoadMap)
        rospy.loginfo('Successful init')                                                            
        rospy.spin()                                                                                # Loop ros comunication
    def get_path(self, path):
        self.path = path
        rospy.loginfo("New path from planner {}".format(self.path.trajectory[0].multi_dof_joint_trajectory.points))
        self.path = self.path.trajectory[0].multi_dof_joint_trajectory.points   #Store planned waypoints
        # Visualize planned path in Rviz
        self.visPath(self.path)
        # Matplot lib visualization


    # Callback function for executing trajectory
    def execute(self, goal): 
        rospy.loginfo("Executing callback for trajectory execution beep boop")
        rospy.sleep(1)
        #Loop through all points in trajectory
        for _point in self.path:
            _w =_point.transforms
            rospy.loginfo("Commanding new waipoint (transforms) {}".format(_w))
            _w = _w[0].translation
            rospy.loginfo("Commanding new waipoint (.translation){}".format(_w))
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
            #for _ in range(3):
            # Publish a few times - publishing just once might fail.
            self.pub.publish(trajectory_execute)
            #rospy.sleep(0.5)
            #Wait until point is reached
            while not self.tol(_w):
                rospy.sleep(0.1)             

        # Reset counter for next trajecotry execution
        #rospy.loginfo("Canceling planning in moveit allowing for replanning")
        #answerCancel = GoalID()
        #answerCancel.stamp = rospy.get_rostime()
        #answerCancel.id = ""
        #self.pubCancelAction.publish(answerCancel)
    
    def get_pose(self, pose):
        self.pose = pose

    def tol(self, current):
        _pose = self.pose.position
        diff_x = (_pose.x - current.x) ** 2
        diff_y = (_pose.y - current.y) ** 2
        diff_z = (_pose.z - current.z) ** 2
        _diff = (diff_x + diff_y + diff_z) ** 0.5
        if _diff <0.2:
            rospy.loginfo("Diff from waypoint is {}, continuing".format(_diff))
            return True
        else:
            rospy.loginfo("Diff from waypoint is {}, waiting".format(_diff))
            return False
        
    def visPath(self, path):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0
        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        for _point in self.path:
            _w =_point.transforms
            _w = _w[0].translation
            marker.pose.position.x = _w.x
            marker.pose.position.y = _w.y
            marker.pose.position.z = _w.z
            self.marker_pub.publish(marker)
            rospy.rostime.sleep(.1)   



if __name__ == '__main__':
    TrajectoryActionController()                                     # Run class constructor on program start 