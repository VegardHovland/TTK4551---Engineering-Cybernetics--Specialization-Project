
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory


class QuadrotorCommander:
    def __init__(self):
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "quadrotor"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    def set_start_position(self, x, y, z):
        # Set the starting pose
        start_pose = PoseStamped()
        start_pose.header.frame_id = "world"
        start_pose.pose.position.x = x
        start_pose.pose.position.y = y
        start_pose.pose.position.z = z
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(start_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_waypoint(self, x, y, z):
        # Set the target pose
        pose_target = PoseStamped()
        pose_target.header.frame_id = "world"
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = z
        self.move_group.set_pose_target(pose_target)

        # Plan and execute the trajectory
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('quadrotor_commander', anonymous=True)

        # Create the MoveIt commander interface
        quadrotor_commander = QuadrotorCommander()
        # Get robot position
        
        # Set the starting position of the quadrotor
        quadrotor_commander.set_start_position(0.0, 0.0, 0.0)

        # Command the quadrotor to a waypoint
        quadrotor_commander.go_to_waypoint(1.0, 2.0, 3.0)

    except rospy.ROSInterruptException:
        pass
