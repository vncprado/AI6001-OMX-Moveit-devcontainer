#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This node will recognize the object and publish the target frame 

import cv2
import rospy
import math
from distutils.version import LooseVersion
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import queue
import tf2_ros
import geometry_msgs.msg
import tf_conversions

# if sys.version_info < (3, 0):
#     PY3 = False
#     import Queue as queue
# else:
#     PY3 = True


boxPoints = cv2.boxPoints if LooseVersion(cv2.__version__) >= LooseVersion('3.0') else cv2.cv.BoxPoints

COLOR_DICT = {
    'red': {'lower': np.array([0, 43, 46]), 'upper': np.array([10, 255, 255])},
    'blue': {'lower': np.array([90, 100, 100]), 'upper': np.array([130, 255, 255])},
    'green': {'lower': np.array([50, 60, 60]), 'upper': np.array([77, 255, 255])},
    'yellow': {'lower': np.array([20, 40, 46]), 'upper': np.array([34, 255, 255])},
}

class GazeboCamera(object):
    def __init__(self, topic_name='/camera/color/image_raw/compressed'):
        self._frame_que = queue.Queue(10)
        self._bridge = CvBridge()
        self._img_sub = rospy.Subscriber(topic_name, CompressedImage, self._img_callback)

    def _img_callback(self, data):
        if self._frame_que.full():
            self._frame_que.get()
        self._frame_que.put(self._bridge.compressed_imgmsg_to_cv2(data))
    
    def get_frame(self):
        if self._frame_que.empty():
            return None
        return self._frame_que.get()

def rect_to_move_params(rect):
    return int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3), int((552 - rect[0][0]) * 900.0 / 460.0 - 450), rect[2] - 90
    
def get_recognition_rect(frame, lower=COLOR_DICT['red']['lower'], upper=COLOR_DICT['red']['upper'], show=True):
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    erode_hsv = cv2.erode(hsv, None, iterations=2)
    inRange_hsv = cv2.inRange(erode_hsv, lower, upper)
    contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rects = []
    for _, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        if rect[1][0] < 10 or rect[1][1] < 10:
            continue
        # print(rect)
        box = boxPoints(rect)
        cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 1)
        rects.append(rect)
    
    if show:
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown('key to exit')
    return rects

if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=False)
    rate = rospy.Rate(1.0)
    
    br = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # color = COLOR_DICT['red']
    color_name = 'green'
    color = COLOR_DICT[color_name]

    cam = GazeboCamera(topic_name='/camera/color/image_raw/compressed')

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        if frame is None:
            continue
        rects = get_recognition_rect(frame, lower=color['lower'], upper=color['upper'])
        if len(rects) == 0:
            continue

        for i in range(len(rects)): # [random.randint(0, 100) % len(rects)]
            rect = rects[i]
            x, y, angle = rect_to_move_params(rect)
            print('target: x={:.2f}mm, y={:.2f}mm, anlge={:.2f}'.format(x, y, angle))
            
            pose_target = geometry_msgs.msg.TransformStamped()
            pose_target.header.frame_id = "world"
            pose_target.child_frame_id = "box" + str(i) + color_name

            pose_target.transform.translation.x = x/1000.0
            pose_target.transform.translation.y = y/1000.0
            pose_target.transform.translation.z = 0.0

            q = tf_conversions.transformations.quaternion_from_euler(0, math.radians(angle), 0)
            pose_target.transform.rotation.x = q[0]
            pose_target.transform.rotation.y = q[1]
            pose_target.transform.rotation.z = q[2]
            pose_target.transform.rotation.w = q[3]

            pose_target.header.stamp = rospy.Time.now()

            br.sendTransform(pose_target)
        
    