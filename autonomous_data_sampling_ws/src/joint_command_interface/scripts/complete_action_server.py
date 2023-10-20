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
from actionlib_msgs.msg import GoalStatusArray, GoalID, GoalStatus
import math
import tf
#from joint_command_interface.action import FollowJointTrajectoryAction, MultiDOFJointTrajectoryAction 
#from joint_command_interface.msg import      
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
#from service_proxies import *


class TrajectoryActionController(object):
    def __init__(self):
        rospy.init_node('rmf_obelix_interface')                                                       # init ros node
        rospy.loginfo("Node has started")
        #self._action_ns = '/action/trajectory'  # controller_name + '/action/trajectory'                              # set namespace, same as moveit wants
        #self._as = actionlib.SimpleActionServer(self._action_ns,MultiDofFollowJointTrajectoryAction,execute_cb=self.execute_cb,auto_start = False)
        #self._as.register_goal_callback(self.goalCB)                                                # Register goal with callback function
        #self._action_name = rospy.get_name()
        #self._as.start()                                                                            # Start actionserver
        #self._feedback = MultiDofFollowJointTrajectoryActionFeedback
        #self._result = MultiDofFollowJointTrajectoryActionResult
        #self.pos = Float32MultiArray()                                                              # Poisiton being published
        #self.speed = Float32MultiArray()                                                            # velocity being published
        #self.i=0                                                                                    # Counter for viapoint looping
        #self.timer = 0
        self.pose = Pose()                                                             # init pos for joint states                                                     
        # Subscriber pose and publish to joint states
        self.sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=self.get_pose)
        self.commanSub = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callback=self.execute)
        # Publish move command
        self.pub = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=1)        #rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        self.pubCancelAction = rospy.Publisher('/move_base/cancel',GoalID,queue_size=0)
        self.statusPub = rospy.Publisher('/execute_trajectory/status',GoalStatusArray,queue_size=0)
        rospy.loginfo('Successful init')                                                            
        rospy.spin()                                                                                # Loop ros comunication

    # Callback function for executing trajectory
    def execute(self, goal): 
        rospy.loginfo("Executing callback for trajectory execution beep boop")
        rospy.sleep(2)

        rospy.loginfo(goal.goal)
        # Set goal to be executed/succeeded
        status = GoalStatusArray()
        status.header = Header()
        status.header.stamp = rospy.Time.now()
        goalstatus = GoalStatus()
        goalstatus.goal_id = goal.goal_id
        goalstatus.status = 3
        status.status_list = [goalstatus]
        #status.status_list = [3]
        for _ in range(3):
            self.statusPub.publish(status)
            rospy.sleep(1)
        #Excetue trajectory
        rospy.loginfo("Position Path points")
        trajectory_points = goal.goal.trajectory                                                 # get the different points of the trajecotry
        rospy.loginfo(trajectory_points)
        rospy.loginfo("Position only Path points")
        trajectory_points = trajectory_points.multi_dof_joint_trajectory.points    #.points.transforms.translation
        rospy.loginfo(trajectory_points)

        #Fufil multidoftraj msg

        
        #Loop through all points in trajectory
        for _point in trajectory_points:
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
            for _ in range(3):
            # Publish a few times - publishing just once might fail.
                self.pub.publish(trajectory_execute)
                rospy.sleep(0.5)
            #Wait until point is reached
            while not self.tol(_w):
                rospy.sleep(0.5)             

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
        if _diff <0.5:
            rospy.loginfo("Diff from waypoint is {}, continuing".format(_diff))
            return True
        else:
            rospy.loginfo("Diff from waypoint is {}, waiting".format(_diff))
            return False
        
if __name__ == '__main__':
    TrajectoryActionController()                                     # Run class constructor on program start 