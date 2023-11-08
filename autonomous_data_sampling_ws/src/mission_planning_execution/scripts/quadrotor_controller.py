#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray

from service_proxies import *

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

    # Load a set of waypoints TODO: replace with optimal sampling locations from compressed sensing technique.
    w1 = Waypoint(-5.0, -2.0, 1.0)
    w2 = Waypoint(5.0, -2.0, 2.0)
    w3 = Waypoint(0.0, 2.0, 1.5)
    w4 = Waypoint(-5.0, 2.0, 2.0)
    waypoints = [w1, w2, w3, w4]
    wp_man = WPManager(waypoints)

    # Publisher to topic which sets a desired waypoint / goal location.
    # Subscriber to monitor the current pose.
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    sub = rospy.Subscriber("/rmf_obelix/ground_truth/pose", Pose, callback=wp_man.update_pose)
    status_sub = rospy.Subscriber("/pci_output_path/status", GoalStatusArray, callback=wp_man.update_status)

    for _w in waypoints:
        # Compose a message containing the waypoint to publish
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x = _w.x
        msg.pose.position.y = _w.y
        msg.pose.position.z = _w.z

        # Publish waypoint to set the goal position:
        for _ in range(3):
            # Publish a few times - publishing just once might fail.
            pub.publish(msg)
            rospy.sleep(1)


        # Call / Trigger go-to-waypoint service
        call_go_to_waypoint()

        rospy.loginfo("Waiting to reach waypoint")
        while not wp_man.check_pose_on_waypoint():
            rospy.sleep(0.5)
            continue
        #while not wp_man.status == 0:
            # Look for a status flag to check instead of manually checking the distance.
        #    rospy.loginfo("wp_man.status: {}".format(wp_man.status))
        #    rospy.sleep(0.5)
        #    continue

    rospy.loginfo("Waypoints have been traversed")

    # Go home and land
    call_homing_trigger()

    rospy.spin()