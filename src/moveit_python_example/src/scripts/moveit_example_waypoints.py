#!/usr/bin/env python3
import sys
import rospy
import copy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import JointPosition
from math import pi


joint_position = None
# Transform to the world frame
def callback_joint_states(msg):
    global joint_position
    joint_position= msg.position
    #print(joint_position)

def create_world_frame_pose(x = 0,z = 0,angle= 0):


    # The orientation and position that is to be changed in the manipulator frame
    # X is parallel to the jaws, Z is perpendicular to the end effector jaw (the green arrow in the manipulator)
    # The rotation on the y-axis corresponds to the angle of the jaw
    listener = tf.TransformListener()
    # Wait for the transformation to be available
    listener.waitForTransform("/end_effector_link", "/world", rospy.Time(), rospy.Duration(4.0))

    rotation = Rotation.from_euler('y', [angle], degrees=True).as_quat()[0]
    pose_in = PoseStamped()
    pose_in.header.frame_id = "/end_effector_link"  # Set the source frame
    pose_in.pose.position.x = x
    pose_in.pose.position.y = 0.00
    pose_in.pose.position.z = z
    pose_in.pose.orientation.x = rotation[0]
    pose_in.pose.orientation.y = rotation[1]
    pose_in.pose.orientation.z = rotation[2]
    pose_in.pose.orientation.w = rotation[3]
    try:
        return listener.transformPose("/world", pose_in)
        rospy.loginfo("Transformed Pose: %s", wpose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to perform pose transformation")
def move_to_desired_pose(group):
    all_joint_names = ["joint1", "joint2", "joint3", "joint4", "gripper"]
    group.set_goal_tolerance(0.1)  # Increase the goal pose tolerance as needed
    group.set_goal_orientation_tolerance(0.1)  # set the goal orientation tolerance as needed

    group.set_path_constraints(None)  # remove any constraints on the path planning you have set earlier
    # Define the desired end-effector pose

    # Traveling in a square
    waypoints = []
    distance = 3
    for x in range(distance):
        wpose = create_world_frame_pose(x*0.02,0,0)
        waypoints.append(copy.deepcopy(wpose.pose))

    for x in range(distance):
        wpose = create_world_frame_pose(distance*0.02,x*0.02,0)
        waypoints.append(copy.deepcopy(wpose.pose))
    for x in range(distance):
        wpose = create_world_frame_pose(distance * 0.02- x* 0.02,  distance * 0.02, 0)
        waypoints.append(copy.deepcopy(wpose.pose))

    for x in range(distance):
        wpose = create_world_frame_pose(0,  distance * 0.02 - x* 0.02, 0)
        waypoints.append(copy.deepcopy(wpose.pose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.001,        # eef_step
                                    0.0)         # jump_threshold

    print(fraction)
    if fraction > 0.8:
        group.execute(plan, wait=True)
    else:
        print("NO EXECUTION")
        group.execute(plan, wait=True)


    # moving the base
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = joint_goal[0] + -pi*10/180
    print(joint_goal)
    print(group.go(joint_goal, wait=True))
    group.stop()

    # move gripper
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    joint_goal = gripper_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = joint_goal[0] - 0.002
    print(joint_goal)
    print(gripper_group.go(joint_goal, wait=True))
    gripper_group.stop()


if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('moveit_example', anonymous=True)

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())
    joint_angle_sub = rospy.Subscriber("/joint_states", JointState, callback_joint_states)
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print("============ Printing robot state")
    # print(robot.get_current_state())
    move_to_desired_pose(group)
