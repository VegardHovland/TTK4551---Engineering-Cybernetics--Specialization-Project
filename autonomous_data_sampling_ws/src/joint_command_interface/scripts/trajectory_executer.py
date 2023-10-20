#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from joint_command_interface.msg import ExecuteDroneTrajectoryAction, ExecuteDroneTrajectoryFeedback, ExecuteDroneTrajectoryResult
import math
# TODO: add rmf oblix spesific command and pose subsriber
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion
from geometry_msgs.msg import Point
import tf
from geometry_msgs.msg import Twist

#from hector_uav_msgs.msg import PoseAction
#from hector_uav_msgs.srv import EnableMotors

MAX_SPEED = 1.5
EPSILON = 1e-4

class TrajectoryActionController:
    def __init__(self, name):
        rospy.loginfo("1 !")
        self.action_name = name
        self.trajectory = []
        self.success = True
        self.executing = False
        self.last_pose = Pose()
        self.feedback_ = ExecuteDroneTrajectoryFeedback()
        self.result_ = ExecuteDroneTrajectoryResult()
        rospy.loginfo("2 !")
        rospy.init_node("trajectory_executor")
        self.pose_sub = rospy.Subscriber("/rmf_obelix/ground_truth/state", Pose, self.poseCallback)
        pub = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
        #sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=wp_man.update_pose)
        self.server_ = actionlib.SimpleActionServer(self.action_name, ExecuteDroneTrajectoryAction, execute_cb=self.executeCB, auto_start=False)
        rospy.loginfo("3 !")
        # Server never init! 
        # self.orientation_client_ = actionlib.SimpleActionClient("/rmf_obelix/ground_truth/pose", ExecuteDroneTrajectoryAction)
        # self.orientation_client_.wait_for_server()

        self.empty = Twist()
        self.cmd = Twist()
        
        rospy.loginfo("4 !")
        self.empty.linear.x = 0
        self.empty.linear.y = 0
        self.empty.linear.z = 0
        self.empty.angular.x = 0
        self.empty.angular.y = 0
        self.empty.angular.z = 0

        self.server_.start()
        rospy.loginfo("Init sucess !")
        rospy.spin() 

    def executeCB(self, goal):
        self.executing = True
        self.trajectory = goal.trajectory
        rospy.loginfo("Executing trajectory!")
        for i in range(len(self.trajectory) - 1):
            if self.server_.is_preempt_requested() or not rospy.is_shutdown():
                rospy.loginfo("Preempt requested")
                self.success = False
                self.executing = False
                break

            last_pose = self.trajectory[i + 1].position
            self.feedback_.current_pose = last_pose
            rospy.sleep(0.05)
            goalx = self.trajectory[i + 1].position.x
            goaly = self.trajectory[i + 1].position.y
            goalz = self.trajectory[i + 1].position.z
            diffx = goalx - last_pose.position.x
            diffy = goaly - last_pose.position.y
            diffz = goalz - last_pose.position.z
            step_angle = math.atan2(diffy, diffx)
            q = euler_from_quaternion([last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z, last_pose.orientation.w])
            heading = q[2]
            rospy.loginfo("Diffz: %lf" % diffz)
            rospy.loginfo("Step angle: %lf, heading: %lf" % (step_angle, heading))
            r = rospy.Rate(4)
            if ((abs(diffx) > 0.01 or abs(diffy) > 0.01) and abs(step_angle - heading) > 0.3):
                goal = MultiDOFJointTrajectory()
                self.orientation_client_.send_goal(goal)
                self.orientation_client_.wait_for_result()
                continue
            while abs(diffz) > 0.08:
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.z = diffz * 3
                self.vel_pub.publish(vel_msg)
                rospy.sleep(0.05)
                diffz = goalz - last_pose.position.z
                self.server_.publish_feedback(self.feedback_)

            latched_distance = math.sqrt(diffx**2 + diffy**2)
            distance = latched_distance
            vel_msg = Twist()
            vel_msg.linear.y = 0
            vel_msg.linear.x = MAX_SPEED
            vel_msg.linear.z = 0
            if i > len(self.trajectory) - 3:  # Slow down at final waypoints.
                vel_msg.linear.x /= 3
            rospy.loginfo("Computed distance: %lf" % distance)
            while distance > 0.4 * MAX_SPEED:
                self.vel_pub.publish(vel_msg)
                rospy.sleep(0.05)
                distance = math.sqrt((goalx - last_pose.position.x)**2 + (goaly - last_pose.position.y)**2)
                rospy.loginfo("Distance to goal: %lf" % distance)
                self.server_.publish_feedback(self.feedback_)
                if distance < latched_distance:
                    latched_distance = distance
                if distance > latched_distance:
                    goal = MultiDOFJointTrajectory()
                    p = Pose()
                    p.position.x = goalx
                    p.position.y = goaly
                    p.position.z = goalz
                    # q = quaternion_from_euler(0, 0, step_angle)
                    p.orientation.x = q[0]
                    p.orientation.y = q[1]
                    p.orientation.z = q[2]
                    p.orientation.w = q[3]
                    self.feedback_.current_pose = p
                    goal.target_pose.pose = p
                    goal.target_pose.header.frame_id = "world"
                    self.orientation_client_.send_goal(goal)
                    self.orientation_client_.wait_for_result()
                    break

        self.executing = False
        if not self.success:
            self.result_.result_code = ExecuteDroneTrajectoryResult.COLLISION_IN_FRONT
            self.server_.set_preempted(self.result_)
        rospy.loginfo("Executed trajectory!")
        self.result_.result_code = ExecuteDroneTrajectoryResult.SUCCESSFUL
        self.server_.set_succeeded(self.result_)

    def idle(self):
        while not rospy.is_shutdown():
            if not self.executing:
                rospy.loginfo("idle !")
            rospy.sleep(0.25)

    def actionCallback(self, feedback, p):
        rospy.loginfo("action callback !")
        euler_distance = (p.position.x - feedback.current_pose.pose.position.x) ** 2 + \
                        (p.position.y - feedback.current_pose.pose.position.y) ** 2 + \
                        (p.position.z - feedback.current_pose.pose.position.z) ** 2
        euler_distance = math.sqrt(euler_distance)
        if euler_distance < 0.15:
            self.orientation_client_.cancel_goal()

    def poseCallback(self, msg):
        rospy.loginfo("pose callback !")
        self.last_pose = msg.pose

if __name__ == "__main__":
    controller = TrajectoryActionController("execute_trajectory")
    controller.idle()
