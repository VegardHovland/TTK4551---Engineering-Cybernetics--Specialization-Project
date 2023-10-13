#! /usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Int32
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
from actionlib_msgs.msg import GoalStatusArray
import math
import tf
#from joint_command_interface.action import FollowJointTrajectoryAction, MultiDOFJointTrajectoryAction 
from joint_command_interface.msg import   MultiDofFollowJointTrajectoryAction, MultiDofFollowJointTrajectoryActionFeedback, MultiDofFollowJointTrajectoryActionGoal, MultiDofFollowJointTrajectoryActionResult, MultiDofFollowJointTrajectoryFeedback, MultiDofFollowJointTrajectoryGoal, MultiDofFollowJointTrajectoryResult 

#from service_proxies import *


class TrajectoryActionController(object):
    def __init__(self, controller_name):
        rospy.init_node('rmf_obelix_interface')                                                       # init ros node
        self._action_ns = '/action/trajectory'  # controller_name + '/action/trajectory'                              # set namespace, same as moveit wants
        self._as = actionlib.SimpleActionServer(self._action_ns,MultiDofFollowJointTrajectoryAction,execute_cb=self.execute_cb,auto_start = False)
        self._as.register_goal_callback(self.goalCB)                                                # Register goal with callback function
        self._action_name = rospy.get_name()
        self._as.start()                                                                            # Start actionserver
        self._feedback = MultiDofFollowJointTrajectoryActionFeedback
        self._result = MultiDofFollowJointTrajectoryActionResult
        self.pos = Float32MultiArray()                                                              # Poisiton being published
        self.speed = Float32MultiArray()                                                            # velocity being published
        self.i=0                                                                                    # Counter for viapoint looping
        self.timer = 0
        self.pose = Pose()                                                             # init pos for joint states                                                     
        # Subscriber pose and publish to joint states
        self.sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=self.get_pose)
        # Publish move command
        pub = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=1)        #rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        rospy.loginfo('Successful init')                                                            
        rospy.spin()                                                                                # Loop ros comunication

    # Callback function for executing trajectory
    def execute_cb(self, goal): 
        rospy.loginfo("Executing callback")
        trajectory_points = goal.trajectory.points                                                  # get the different points of the trajecotry
        self.viapoints = trajectory_points                                                          # store as viapoints
        self._goal = self._as.set_succeeded()                                                       # Sucessfully got viapoints, sending sucess to moveit. Rest is on hardware level
        self.i = 0                                                                                  # Force reset
        self.timer = time.time()                                                                    # start timer
        # Fufill trajectory msg
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = 'world'
        traj.header.stamp.secs = rospy.Time.now().secs
        traj.header.stamp.nsecs = rospy.Time.now().nsecs
        traj.joint_names = []
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(0))
        while self.i < len(self.viapoints):                                                         # Loop over all the viapoints
            _wx = self.viapoints[self.i].positions[0]
            _wy = self.viapoints[self.i].positions[1]
            _wz = self.viapoints[self.i].positions[2]     
            transforms =Transform(translation=Point(_wx, _wy, _wz), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
            velocities = Twist()        # Empty velocity msg
            accelerations = Twist()     # Emptry acceleration msg
            point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
            point.time_from_start.secs = 4
            point.time_from_start.nsecs = rospy.Time.now().nsecs + 100000000 + self.i*50000000
            traj.points.append(point)
            rospy.spin                                                                              # Read new pos from sensors
            time.sleep(0.02)                                                                        # sleep long enough to get new joint_states
            rospy.sleep                                                                             # continue program
            # Publish waypoint to set the goal position:
            rospy.loginfo("Publishing new viapoint")
            for _ in range(3):
                self.pub.publish(traj)
            rospy.sleep(1)
            # Continue when reached setpoint
            if self.tol():                                                                          # Check if close enough to setpoint
                self.i = self.i + 1                                                                 # loop to next setpoints in path  
            
            # Exit if not completed within 50 sec
            if (time.time()- self.timer) > 50:                                                  
                break

        rospy.loginfo("Executing callback finished, setting succeded")
        self._goal = self._as.set_succeeded()                  
        self.speed.data = self.viapoints[self.i - 1].velocities                                     # set speed for end point
        self.i=0                                                                                    # Reset counter for next trajecotry execution

    def goalCB(self):                                                                               # Goal callback function, implement this if motion is planned from continous
        self._goal = self._as.accept_new_goal()                                                     #  Accept next goal      
        # Do something with goal, not used
    
    def get_pose(self, pose):
        self.pose = pose

    def tol(self):
        _pose = self.pose.position
        diff_x = (_pose.x - self.current.x) ** 2
        diff_y = (_pose.y - self.current.y) ** 2
        diff_z = (_pose.z - self.current.z) ** 2
        _diff = (diff_x + diff_y + diff_z) ** 0.5
        if _diff <1:
            rospy.loginfo("Diff from waypoint is {}, continuing".format(_diff))
            return True
        else:
            rospy.loginfo("Diff from waypoint is {}".format(_diff))
            return False
        
if __name__ == '__main__':
    JointTrajectoryActionServer('rmf_obelix')                                     # Run class constructor on program start 