#!/usr/bin/python3
import rospy
from std_srvs.srv import Trigger
from voxblox_msgs.srv import FilePath
from planner_msgs.srv import planner_string_trigger, pci_initialization

''' 
    ROS SERVICE PROXIES
    Enables calling / sending requests and receiving responses from existing SERVICES. 

    Format:
    from name_of_ros_package.srv import Type
    def call_name_of_service(request_message: ServiceMsg):
    try:
        wait for the service to become available
        create a ServerProxy("/name/of/service", ServiceClass)
        call_service(request_message)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
'''


def call_load_map(map_path: str):
    try:
        rospy.wait_for_service("/gbplanner_node/load_map")
        load_map = rospy.ServiceProxy("/gbplanner_node/load_map", FilePath)
        response = load_map(map_path)
    except rospy.ServiceException as e:
        rospy.logwarn(e)


def call_load_graph(graph_path: str):
    '''

    '''
    try:
        rospy.wait_for_service("/gbplanner/load_graph")
        load_map = rospy.ServiceProxy("/gbplanner/load_graph", planner_string_trigger)
        response = load_map(graph_path)
    except rospy.ServiceException as e:
        rospy.logwarn(e)


def call_initialization():
    '''
    Triggers initialization movement
    '''
    try:
        rospy.wait_for_service("/pci_initialization_trigger")
        init = rospy.ServiceProxy("/pci_initialization_trigger", pci_initialization)
        response = init()
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    return


def call_go_to_waypoint():
    '''
    Calls (triggers) the "go to waypoint" service, which causes the quadrotor to navigate to the set waypoint.
    Requires that a waypoint has been published to the topic [/move_base_simple/goal] first.
    '''
    try:
        rospy.wait_for_service("/planner_control_interface/std_srvs/go_to_waypoint")
        go_to_waypoint = rospy.ServiceProxy("/planner_control_interface/std_srvs/go_to_waypoint", Trigger)
        response = go_to_waypoint()
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    return


def call_homing_trigger():
    '''
    Triggers the PCI's homing service, sending the quadrotor drone back to start.
    '''
    try:
        rospy.wait_for_service("/planner_control_interface/std_srvs/homing_trigger")
        homing_trigger = rospy.ServiceProxy("/planner_control_interface/std_srvs/homing_trigger", Trigger)
        response = homing_trigger()
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    return

