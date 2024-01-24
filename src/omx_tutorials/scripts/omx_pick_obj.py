#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('omx_move_group_tutorial', anonymous=True)

rate = rospy.Rate(1.0)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.move_group.MoveGroupCommander('gripper')

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = arm_group.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = arm_group.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print("============ Printing robot state")
# print(robot.get_current_state())
# print("")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

def gripper_open():
    # open gripper
    gripper_joint = gripper_group.get_current_joint_values()
    gripper_joint[0] = 0.008
    gripper_group.set_joint_value_target(gripper_joint)
    ret = gripper_group.go(wait=True)
    print('Gripper open, ret={}'.format(ret))

    return ret

def gripper_close():
    # close gripper
    gripper_joint = gripper_group.get_current_joint_values()
    gripper_joint[0] = 0.002
    gripper_group.set_joint_value_target(gripper_joint)
    ret = gripper_group.go(wait=True)
    print('Gripper open, ret={}'.format(ret))

    return ret

def set_joints(move_group, angles, wait=True):
    try:
        joint_goal = move_group.get_current_joint_values()

        for i in range(len(joint_goal)):
            if i >= len(angles):
                break
            if angles[i] is not None:
                joint_goal[i] = math.radians(angles[i])

        print('set_joints, joints={}'.format(joint_goal))
        move_group.set_joint_value_target(joint_goal)
        ret = move_group.go(wait=wait)
        print('move to finish, ret={}'.format(ret))
        return ret
    except Exception as e:
        print('[Ex] set_joints exception, ex={}'.format(e))

def go_to_obj():

    # go to object position
    try:
        object_frame = tfBuffer.lookup_transform('world', 'box0green', rospy.Time())

        waypoints = []

        wpose = arm_group.get_current_pose().pose
        # wpose.position.z -= scale * 0.1  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position = object_frame.transform.translation
        pose_goal.orientation.x = 0.0 #object_frame.transform.rotation
        pose_goal.orientation.y = 0.0 #object_frame.transform.rotation
        pose_goal.orientation.z = 0.0 #object_frame.transform.rotation
        pose_goal.orientation.w = 1.0 #object_frame.transform.rotation
        # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(pose_goal))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        print(f"Fraction planned: {fraction}")

        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position = object_frame.transform.translation
        # pose_goal.orientation.x = 0.0 #object_frame.transform.rotation
        # pose_goal.orientation.y = 0.0 #object_frame.transform.rotation
        # pose_goal.orientation.z = 0.0 #object_frame.transform.rotation
        # pose_goal.orientation.w = 1.0 #object_frame.transform.rotation
        # # pose_goal.orientation.w = 1.0
        # # pose_goal.position.x = 0.4
        # # pose_goal.position.y = 0.1
        # # pose_goal.position.z = 0.4
        # group.set_pose_target(pose_goal)

        # plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        # group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        arm_group.execute(plan, wait=True)
        # group.clear_pose_targets()
        return fraction
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        return 0.0

while not rospy.is_shutdown():
    
    # Go to home pose
    set_joints(arm_group, [0.0, 0.0, 0.0, 0.0])
    gripper_open()
    print("Ready to plan.")

    trajectory_fraction = go_to_obj()

    if trajectory_fraction > 0.8:
        print("Closing gripper")
        gripper_close()
    
    rate.sleep()
    # # Go to home pose
    # set_joints(arm_group, [0.0, 0.0, 0.0, 0.0])