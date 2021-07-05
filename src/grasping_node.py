#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import time

import tf
import tf2_ros
import tf2_geometry_msgs

import moveit_commander

from std_msgs.msg import Float32, Int64
from grasping_tp.msg import DetectedObj, BoundingBoxCoord, GraspServerRequest, GraspConfigList, GraspConfig
from grasping_tp.srv import object_detection, detect_grasps
from geometry_msgs.msg import Pose

from utils import *

import numpy as np

class Grasping():
    def __init__(self):
        rospy.init_node("grasping_node")
        self.listener = tf.TransformListener()

    def detect_objects(self):
        rospy.wait_for_service('/object_detection')
        detect_service = rospy.ServiceProxy('/object_detection', object_detection)
        msg = Int64(4)
        resp = ""
        start = time.time()
        
        try:
            resp = detect_service(msg)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        while(resp.detected_objects.object_names[0].data == "nothing"):
            try:
                resp = detect_service(msg)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            if time.time() >= start+2:
                return False, False, False

        detected_obj = DetectedObj()
        detected_obj = resp.detected_objects

        ind = 0
        return detected_obj.objects_bb[ind], detected_obj.object_names[ind].data, detected_obj.cloud
    
    def get_grasp_conf(self, bb, pc):
        rospy.wait_for_service('/grasping')
        grasp_service = rospy.ServiceProxy('/grasping', detect_grasps)

        msg = GraspServerRequest()
        msg.bounding_box = bb
        msg.global_cloud = pc

        start = time.time()

        try:
            print("Ask grasping service for config")
            resp2 = grasp_service(msg)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

        print("Time to find a good grasp: {}".format(time.time() - start))

        detected_grasp = GraspConfigList()
        detected_grasp = resp2.grasp_configs.grasps
        best_grasp = GraspConfig()
        best_grasp = detected_grasp[0]
        return best_grasp
    
    def move_to(self, pose):
		arm.allow_replanning(True)
		arm.set_num_planning_attempts(5)
		arm.set_workspace([-3.0, -3.0, 3.0, 3.0])
		arm.set_planning_time(10)
		pose_to_perform = arm.get_current_pose()
        
		pose_to_perform.header.frame_id = "/odom"
		pose_to_perform.pose.position = pose.position
		pose_to_perform.pose.orientation = pose.orientation
        
		#arm.set_pose_target(pose_to_perform)
		arm.set_joint_value_target(pose_to_perform, "hand_palm_link", True)

		plan = arm.go()
		# Calling `stop()` ensures that there is no residual movement
		arm.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		arm.clear_pose_targets()
    
    def grasp(self, grasp_conf):
        print("---------")
        print("GRASP")
        print("---------")
        begin_pose = arm.get_current_pose().pose
        move_hand(1)

        if begin_pose.position.z < grasp_conf.pre_pose.position.z:
            print("Set z")
            z_pose = begin_pose = arm.get_current_pose().pose
            z_pose.position.z = grasp_conf.pre_pose.position.z + 0.025
            self.move_to(z_pose)
            

        print("Prepose")
        self.move_to(grasp_conf.pre_pose)
        print("Actual pose")
        self.move_to(grasp_conf.actual_pose)

        move_hand(0)

        print("Come back")
        self.move_to(begin_pose)
    
    def drop(self):
        print("---------")
        print("DROP")
        print("---------")

        x = 0.3
        a = 180

        begin_pose = arm.get_current_pose().pose
        print("Turn left")
        move_base_vel(0, 0, a)
        pose = arm.get_current_pose().pose
        pose.position.x += x
        print("Arm forward")
        self.move_to(pose)


        print("Drop")
        move_hand(1)
        move_hand(0)

        print("Arm backward")
        move_arm_init()
        print("Turn back")
        move_base_vel(0, 0, -a)
        
    def work(self):
        print("Ask detection service for objects")
        bb, name, pc = self.detect_objects()
        if name:
            print("Trying to grasp {}".format(name))
            grasp_conf = self.get_grasp_conf(bb, pc)
            self.grasp(grasp_conf)
            self.drop()
            self.work()

    def start(self):
        move_arm_init()

        self.work()
            


if __name__ == "__main__":
    grasping = Grasping()
    grasping.start()