#! /usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray,Header
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryGoal
)
#from trajectory_msgs.msg import (JointTrajectoryPoint)
from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray
#from service_proxies import *


class JointTrajectoryActionServer(object):
    def __init__(self, controller_name):
        rospy.init_node('robotleg_interface')                                                       # init ros node
        self._action_ns = controller_name + '/follow_joint_trajectory'                              # set namespace, same as moveit wants
        self._as = actionlib.SimpleActionServer(self._action_ns,FollowJointTrajectoryAction,execute_cb=self.execute_cb,auto_start = False)
        self._as.register_goal_callback(self.goalCB)                                                # Register goal with callback function
        self._action_name = rospy.get_name()
        self._as.start()                                                                            # Start actionserver
        self._feedback = FollowJointTrajectoryFeedback
        self._result = FollowJointTrajectoryResult
        self.pos = Float32MultiArray()                                                              # Poisiton being published
        self.speed = Float32MultiArray()                                                            # velocity being published
        self.i=0                                                                                    # Counter for viapoint looping
        self.timer = 0
        self.pose = Pose()                                                             # init pos for joint states                                                     
        # Subscriber pose and publish to joint states
        self.sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=self.get_pose)
        # publisher join_states
        self.pub = rospy.Publisher('/rmf_obelix/joint_states', JointState, queue_size=10)
        # Publish move command
        self.pub_move = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        #rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        rospy.loginfo('Successful init')                                                            
        rospy.spin()                                                                                # Loop ros comunication

    # Callback function for executing trajectory
    def execute_cb(self, goal): 
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        #msg.pose.position.x = _w.x
        #msg.pose.position.y = _w.y
        #msg.pose.position.z = _w.z
        #pub.publish(msg)
        joint_names = goal.trajectory.joint_names                                                   # Get joint names
        trajectory_points = goal.trajectory.points                                                  # get the different points of the trajecotry
        self.viapoints = trajectory_points                                                          # store as viapoints
        self._goal = self._as.set_succeeded()                                                       # Sucessfully got viapoints, sending sucess to moveit. Rest is on hardware level
        self.i = 0                                                                                  # Force reset
        self.timer = time.time()                                                                    # start timer
        while self.i < len(self.viapoints):                                                         # Loop over all the viapoints
            #for j in range(4): 
            msg.pose.position.x = self.viapoints[self.i].positions[0]
            msg.pose.position.y = self.viapoints[self.i].positions[1]
            msg.pose.position.z = self.viapoints[self.i].positions[2]                                                                    # loop over 4 joints
                #self.speed.data[j] = self.viapoints[self.i].velocities[j]                           # Update current velocity
                #if self.i == (len(self.viapoints) - 1):                                             # Speed for last point is 0, avoid stopping to early
                    #self.speed.data[j] = self.viapoints[self.i -1].velocities[j]                    # Save speed from last-1 point
            rospy.spin                                                                              # Read new pos from sensors
            time.sleep(0.02)                                                                        # sleep long enough to get new joint_states
            rospy.sleep                                                                             # continue program
            # Publish waypoint to set the goal position:
            for _ in range(3):
                # Publish a few times - publishing just once might fail.
                self.pub_move.publish(msg)
            rospy.sleep(1)
            if self.tol():                                                                          # Check if close enough to setpoint
                self.i = self.i + 1                                                                 # loop to next setpoints in path  
            if (time.time()- self.timer) > 5:                                                      # exit if not completed within 10 sec
                break
        #self._goal = self._as.set_succeeded()                  
        #self.speed.data = self.viapoints[self.i - 1].velocities                                     # set speed for end point
        self.i=0                                                                                    # Reset counter for next trajecotry execution

    def goalCB(self):                                                                               # Goal callback function, implement this if motion is planned from continous
        self._goal = self._as.accept_new_goal()                                                     #  Accept next goal      
        # Do something with goal, not used

    def get_pose(self, pose):          
        self.pose = pose
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['virtual_joint/trans_x', 'virtual_joint/trans_y', 'virtual_joint/trans_z', 'virtual_joint/rot_x', 'virtual_joint/rot_y', 'virtual_joint/rot_z', 'virtual_joint/rot_w']
        joint_state.position = [self.pose.position.x, self.pose.position.y, self.pose.position.z, 0,0,0,1]
        joint_state.velocity = []
        joint_state.effort = []
        self.pub.publish(joint_state)
        #self.jointStates = msg.position   
        #self.jointVel =   msg.velocity                                                          # Get joint states
    
    def tol(self):
        tolerance = True
        for indeks in range(4):
         if abs(self.pos.data[indeks] - self.jointStates[indeks]) > 0.035:                          # Check if within tolerance curently 2 degrees
            tolerance = False
        return tolerance
        
if __name__ == '__main__':
    JointTrajectoryActionServer('rmf_obelix')                                     # Run class constructor on program start 