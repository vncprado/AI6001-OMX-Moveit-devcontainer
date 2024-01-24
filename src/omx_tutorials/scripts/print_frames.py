#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf2_ros
import rospy
from tf.transformations import quaternion_matrix
import numpy as np

rospy.init_node('omx_move_group_tutorial', anonymous=True)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

def print_homogeneous_matrix(object_frame, frame_name=""):
    # Assuming object_frame.transform.rotation is a geometry_msgs.msg.Quaternion
    rot_matrix = quaternion_matrix([object_frame.transform.rotation.x,
                                    object_frame.transform.rotation.y,
                                    object_frame.transform.rotation.z,
                                    object_frame.transform.rotation.w])
    
    # Extract translation vector
    translation = np.array([object_frame.transform.translation.x,
                            object_frame.transform.translation.y,
                            object_frame.transform.translation.z])
    
    # Create a homogeneous transformation matrix
    homogeneous_matrix = np.identity(4)
    homogeneous_matrix[:3, :3] = rot_matrix[:3, :3]
    homogeneous_matrix[:3, 3] = translation
    
    # Format the output to three decimal points
    np.set_printoptions(precision=3, suppress=True)

    print(f"Homogeneous Transformation Matrix ({frame_name}):\n", homogeneous_matrix)

while not rospy.is_shutdown():
    transform_available = tfBuffer.can_transform('world', 'box0green', rospy.Time(), timeout=rospy.Duration(1.0))
    if transform_available:
        object_frame = tfBuffer.lookup_transform('world', 'box0green', rospy.Time())
        print_homogeneous_matrix(object_frame, frame_name="world->box0green")
        
    transform_available = tfBuffer.can_transform('world', 'camera_link', rospy.Time(), timeout=rospy.Duration(1.0))
    if transform_available:
        object_frame = tfBuffer.lookup_transform('world', 'camera_link', rospy.Time())
        print_homogeneous_matrix(object_frame, frame_name="world->camera_link")
    
    transform_available = tfBuffer.can_transform('camera_link', 'box0green', rospy.Time(), timeout=rospy.Duration(1.0))
    if transform_available:
        object_frame = tfBuffer.lookup_transform('camera_link', 'box0green', rospy.Time())
        print_homogeneous_matrix(object_frame, frame_name="camera_link->box0green")
    break