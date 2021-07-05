#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import os

os.environ["DARKNET_PATH"] = "/workspace/src/dependencies/darknet/"

import rospy

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointCloud, PointField
from geometry_msgs.msg import Point32, Point, PoseStamped, Pose, Twist

import time

import cv2
from cv_bridge import CvBridge, CvBridgeError

import darknet
import random

from std_msgs.msg import String, Header, Int64
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import message_filters

from std_msgs.msg import Float32, Int64
from grasping_tp.msg import DetectedObj, BoundingBoxCoord
from grasping_tp.srv import object_detection


class ObjectsDetection():
    def __init__(self):
        rospy.init_node('ObjectsDetectionNode', anonymous=False)

        self.darknet_config_dir = os.path.join(os.environ.get('DARKNET_PATH', './'), "darknet_config/")
        self.config_file = self.darknet_config_dir+"yolov4-tiny-obj.cfg"
        self.data_file = self.darknet_config_dir+"obj.data"
        self.weights = self.darknet_config_dir+"yolov4-tiny-obj_last.weights"
        random.seed(3)  # deterministic bbox colors
        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.config_file,
            self.data_file,
            self.weights,
            batch_size=1
        )

        self.bridge = CvBridge()

        # SUBSCRIBERS
        self.image_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)
        self.pointcloud_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)

    def service_node(self):
        s = rospy.Service('/object_detection', object_detection, self.handle_object_detection)
        rospy.loginfo("Object Detection Service Node: Waiting for Request...")
        rospy.spin()

    def handle_object_detection(self, req):
        image_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)
        pointcloud_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)

        image_name = self.bridge.imgmsg_to_cv2(image_sub)

        image, detections = self.image_detection(image_name, self.network, self.class_names, self.class_colors)
        darknet.print_detections(detections, False)
        
        labels = []
        bounding_boxes = []
        threshold = 40.0
        
        for obj in detections:
            if (float(obj[1]) > threshold): #Check if the confidence is above a threshold
                labels.append(String(obj[0]))
                x, y, w, h = obj[2][0], obj[2][1], obj[2][2], obj[2][3]

                boundingbox = BoundingBoxCoord()
                x_min, y_min, x_max, y_max = self.convertBack(x,y,w,h)
                boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in self.convertBack(x,y,w,h))

                bounding_boxes.append(boundingbox)

        if not labels:
            final_msg = DetectedObj()
            final_msg.object_names = [String("nothing")]
            return final_msg

        final_msg = DetectedObj()
        final_msg.object_names = labels
        final_msg.objects_bb = bounding_boxes
        final_msg.cloud = pointcloud_sub
        return final_msg

    def image_detection(self, image, network, class_names, class_colors, thresh=0.25):
        # Darknet doesn't accept numpy images.
        # Create one with image we reuse for each detect
        width = darknet.network_width(network)
        height = darknet.network_height(network)

        darknet_image = darknet.make_image(width, height, 3)

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(
            network, class_names, darknet_image, thresh=thresh)
        darknet.free_image(darknet_image)
        image = darknet.draw_boxes(detections, image_resized, class_colors)
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), detections

    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    
if __name__ == "__main__":
    obj_detection_node = ObjectsDetection()
    obj_detection_node.service_node()