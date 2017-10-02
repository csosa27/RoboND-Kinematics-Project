#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
from numpy import array

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *




        
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
###################### Transformation and Rotation Matrix Generation functions ######################
        def TF_create(alpha, a, d, q):
            TF = Matrix([[            cos(q),            -sin(q),           0,             a],
                         [ sin(q)*cos(alpha), cos(q1)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                         [ sin(q)*sin(alpha), cos(q1)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                         [                 0,                  0,           0,             1]])
            return TF

        # Roll, pitch, yaw rotation setup matrix
        def rot_x(r):
            Rotx = Matrix([[1,       0,       0],
                           [0,  cos(r), -sin(r)],
                           [0,  sin(r),  cos(r)]]) # roll
            return Rotx

        def rot_y(p):
            Roty = Matrix([[cos(p),  0, -sin(p)],
                           [0,       1,       0],
                           [-sin(p), 0,  cos(p)]]) # pitch
            return Roty

        def rot_z(y:
            Rotz = Matrix([[cos(y), -sin(y),  0],
                           [sin(y),  cos(y),  0],
                           [0,            0,  1]]) # yaw
            return Rotz
#####################################################################################################

################################## DH parameter data to be stored ###################################
            # joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
	        # DH parameters symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angles

        DH = {alpha0:     0,  a0:      0,  d1:  0.75,   q1:      q1,
              alpha1: -pi/2,  a1:   0.35,  d2:     0,   q2: q2-pi/2,
              alpha2:     0,  a2:   1.25,  d3:     0,   q3:      q3,
              alpha3: -pi/2,  a3: -0.054,  d4:  1.50,   q4:      q4,
              alpha4:  pi/2,  a4:      0,  d5:     0,   q5:      q5,
              alpha5: -pi/2,  a5:      0,  d6:     0,   q6:      q6,
              alpha6:     0,  a6:      0,  d7: 0.303,   q7:       0}
        
		
###################################### Matrix Solver class #########################################
   # Composition of Homogeneous Transforms

        T0_1 = TF_create(alpha0, a0, d1, q1).subs(DH) # base_link to link1
        T1_2 = TF_create(alpha1, a1, d2, q2).subs(DH) # link1 to link2
        T2_3 = TF_create(alpha2, a2, d3, q3).subs(DH) # link2 to link3
        T3_4 = TF_create(alpha3, a3, d4, q4).subs(DH) # link3 to link4
        T4_5 = TF_create(alpha4, a4, d5, q5).subs(DH) # link4 to link5
        T5_6 = TF_create(alpha5, a5, d6, q6).subs(DH) # link5 to link6
        T6_G = TF_create(alpha6, a6, d7, q7).subs(DH) # link6 to link_gripper
		
        T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G) # Transformation Matrix
        

            # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
			
            joint_trajectory_point = JointTrajectoryPoint()
	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            r, p, y = symbols('r p y')
			
            R_corr = rot_z(radians(180))*rot_y(radians(-90)) # Rotational Correction Matrix
			
            R_rpy = rot_x(r)*rot_y(p)*rot_z(y)* R_corr # Rotational Matrix for roll, pitch and yaw

			
            R_rpy = R_rpy.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (0.303) * R_rpy[:,2]


            theta1 = atan2(WC[1],WC[0]) # atan2(yc,xc)

            # Sides for Joint 1, 2 and WC triangle
            r = sqrt(pow(WC[0],2)+pow(WC[1],2)) - .35
            S = WC[2] - 0.75
            s_a = 1.501
            s_b = sqrt(pow(r, 2) + pow(S, 2))
            s_c = 1.25

            # Angles alpha, beta, and gamma (a, b, c)
            ang_a = acos((pow(s_b,2) + pow(s_c,2) - pow(s_a,2))/ (2*s_b*s_c))
            ang_b = acos((pow(s_a,2) + pow(s_c,2) - pow(s_b,2))/ (2*s_a*s_c))
            ang_c = acos((pow(s_a,2) + pow(s_b,2) - pow(s_c,2))/ (2*s_a*s_b))


            theta2 = pi/2 - ang_a - atan2(S, r)
            theta3 = pi/2 - (ang_b + 0.036) # a3/d4 (offset/distance)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3.inv("ADJ") * R_rpy

            #Euler angles - Rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(pow(R3_6[0,2],2) + pow(R3_6[2,2],2)),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	        #
	        # Calculate joint angles using Geometric IK method
	        #
	        #
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
