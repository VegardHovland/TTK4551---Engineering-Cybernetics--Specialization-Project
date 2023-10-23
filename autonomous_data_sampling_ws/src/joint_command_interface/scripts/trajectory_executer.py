#!/usr/bin/env python3
#import rospy
#import actionlib
#from geometry_msgs.msg import Twist, Pose
#from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
#from joint_command_interface.msg import ExecuteDroneTrajectoryAction, ExecuteDroneTrajectoryFeedback, ExecuteDroneTrajectoryResult
#import math
## TODO: add rmf oblix spesific command and pose subsriber
#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
#from geometry_msgs.msg import Transform, Quaternion
#from geometry_msgs.msg import Point
#import tf
#from geometry_msgs.msg import Twist

#from hector_uav_msgs.msg import PoseAction
#from hector_uav_msgs.srv import EnableMotors
import rospy
import actionlib
from threading import Thread
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from moveit_msgs.msg import ExecuteTrajectoryActionFeedback, ExecuteTrajectoryActionResult, ExecuteTrajectoryAction
MAX_SPEED = 1.5
EPSILON = 1e-4

class TrajectoryActionController:
    def __init__(self, name):
        rospy.init_node('robotleg_action_interface')
        #self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointTrajectory, self.process_command)
        #self.state_pub = rospy.Publisher(self.controller_namespace + '/state', FollowJointTrajectoryFeedback, queue_size=1)
        self.action_server = actionlib.SimpleActionServer(name + '/follow_joint_trajectory',
                                                          ExecuteTrajectoryAction,
                                                          execute_cb=self.executeCB,
                                                          auto_start=False)
        #self.action_server.register_goal_callback(self.goalCB)                                                # Register goal with callback function
        self._action_name = rospy.get_name()
        self.action_server.start() 
        #while not self.action_server.is_active():
        #    rospy.loginfo("Action server not yet started")
        #    rospy.sleep(0.1)
        #if self.action_server.is_active():
        #    rospy.loginfo("Action server started successfully")

    def executeCB(self, goal):
        msg = 'Trajectory execution successfully beeing completed'
        rospy.loginfo(msg)
        res = ExecuteTrajectoryActionResult()  
        res.error_code=ExecuteTrajectoryActionResult.SUCCESSFUL
        self.action_server.set_succeeded(result=res, text=msg)


if __name__ == "__main__":
    controller = TrajectoryActionController("multi_dof_joint_trajectory_action")
    #controller.idle()
