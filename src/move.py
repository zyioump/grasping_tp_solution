#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from utils import *

def demo_grasping():
    put_object("cracker_box", 0.6, 1.9, 0.5)
    put_object("mustard_bottle", 1.0, 1.8, 0.5)
    put_object("mug", 0.8, 1.7, 0.5)
    put_object("foam_brick", 1.2, 1.7, 0.5)

	
	# Move toward the table
	# look down a little
    move_head_tilt(-0.5)

	# move position
    move_arm_init()
    move_base_goal(1, 1, 95)
    
if __name__=='__main__':
    rospy.init_node('move')

    demo_grasping()
 