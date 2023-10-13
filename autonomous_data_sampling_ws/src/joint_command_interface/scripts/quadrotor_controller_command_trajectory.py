#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from service_proxies import *
from geometry_msgs.msg import Transform, Quaternion
from geometry_msgs.msg import Point
import tf
import math 
from geometry_msgs.msg import Twist

class Waypoint(object):
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class WPManager(object):
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current = waypoints[0]
        self.current_idx = 0
        self.done = False

        self.pose = Pose()
        self.status = GoalStatusArray()

    def update_pose(self, pose):
        self.pose = pose

    def update_status(self, status):
        self.status = status

    def check_pose_on_waypoint(self):
        _pose = self.pose.position
        diff_x = (_pose.x - self.current.x) ** 2
        diff_y = (_pose.y - self.current.y) ** 2
        diff_z = (_pose.z - self.current.z) ** 2
        _diff = (diff_x + diff_y + diff_z) ** 0.5
        if _diff < 2.5:
            self.advance()
            return True
        else:
            rospy.loginfo("Diff from waypoint is {}".format(_diff))
            return False

    def advance(self):
        if self.current_idx >= len(self.waypoints)-1:
            self.done = True
            return
        self.current_idx += 1
        self.current = waypoints[self.current_idx]


if __name__ == '__main__':
    # Initialize the ROS node. This notifies the ROS MASTER of its existence.
    rospy.init_node("quadrotor_controller")
    rospy.loginfo("Node has started")

    # Load exploration graph
    #graph_path = "~/gbplanner2_ws/my_dt_graph2.gph"
    graph_path = "my_dt_graph.gph"
    call_load_graph(graph_path)
    rospy.loginfo("Loaded graph")

    # Load voxblox volumetric map
    #map_path = "~/gbplanner2_ws/my_dt_map.vxblx"
    map_path = "my_dt.vxblx"
    call_load_map(map_path)
    rospy.loginfo("Loaded map")

    # Initialize quadrotor TODO: run only if drone is not initialized already.
    call_initialization()
    rospy.loginfo("Initing drone with gb palanner, sleeping this ")
    rospy.sleep(10)
    rospy.loginfo("Finished sleeping, starting waypoint traversing ")
    # Load a set of waypoints TODO: replace with optimal sampling locations from compressed sensing technique.
    w1 = Waypoint(-1.0, -1.0, -1.0)
    w2 = Waypoint(-2.0, 1.0, 1.0)
    w3 = Waypoint(-3.0, -1.0, -2.0)
    w4 = Waypoint(-4.0, 2.0, 2.0)
    waypoints = [w1, w2, w3, w4]
    #waypoints = [w1] #  w2, w3, w4]
    wp_man = WPManager(waypoints)

    # Publisher to topic which sets a desired waypoint / goal location.
    # Subscriber to monitor the current pose.
    pub = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=wp_man.update_pose)
    status_sub = rospy.Subscriber("/pci_output_path/status", GoalStatusArray, callback=wp_man.update_status)
    #Desired yaw to go to waypoint with
    quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(0))
    # Create a trajectory message

    for _w in waypoints:
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = 'world'
        trajectory.header.stamp.secs = rospy.Time.now().secs
        trajectory.header.stamp.nsecs = rospy.Time.now().nsecs
        trajectory.joint_names = []
        # Compose a message containing the waypoint to publish
        transforms =Transform(translation=Point(_w.x, _w.y, _w.z), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        velocities = Twist()        # Empty velocity msg
        accelerations = Twist()     # Emptry acceleration msg
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
        point.time_from_start.secs = 4
        point.time_from_start.nsecs = rospy.Time.now().nsecs + 100000000 + waypoints.index(_w)*50000000
        trajectory.points.append(point)
        #Publish trajecotry
        for _ in range(3):
            # Publish a few times - publishing just once might fail.
            pub.publish(trajectory)
            rospy.sleep(1)
        rospy.loginfo("Waypoints traversing started")
        rospy.sleep(2)
        rospy.spin()
        
wp_man.check_pose_on_waypoint()
# Go home and land
# call_homing_trigger() No homing, need to start planner from current point mabye?    
rospy.loginfo("Waypoints traversing finished")
rospy.spin()