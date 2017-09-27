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
from time import time

ERR_ANALYSIS = False

class ErrorAnalysis:
    """ErrorAnalysis

    Attributes:
        ee_cal_pos: list of End-Effector calculate position from computation for the plan
        ee_act_pos: list of End-Effector actual position from request for the plan
        len_req_pose: length of eef-poses from the plan
        ee_offset: list of end-effector offsets for the plan
    """
    def __init__(self, len_req_pose = 0):
        self.ee_cal_pos = []
        self.ee_act_pos = []
        self.len_req_pose = len_req_pose
        self.ee_offset = []

    def append_ee_positions(self,EE_Cal_Pos,EE_Act_Pos):
        self.ee_cal_pos.append(EE_Cal_Pos)
        self.ee_act_pos.append(EE_Act_Pos)

    def print_error_analysis_data(self):
        if (self.len_req_pose != len(self.ee_cal_pos)) or (self.len_req_pose != len(self.ee_cal_pos)):
            rospy.logerr("Error printing error analysis data")
            return None
        
        rospy.loginfo("Printing %s end-effector offsets:" % self.len_req_pose)
        for idx in range(self.len_req_pose):
            print("Pose %02d - Overall End-Effector offset is: %.15f units" % (idx+1,self.ee_offset[idx]))

        rospy.loginfo("Printing %s end-effector (cal,act) positions:" % self.len_req_pose)
        for idx in range(self.len_req_pose):
            print(self.ee_cal_pos[idx])
            print(self.ee_act_pos[idx])

    def root_mean_square_error(self,EE_Cal_Pos,EE_Act_Pos):
        if len(EE_Cal_Pos) == 3 and len(EE_Act_Pos) == 3:
            ee_x_e = abs(EE_Cal_Pos[0]-EE_Act_Pos[0])
            ee_y_e = abs(EE_Cal_Pos[1]-EE_Act_Pos[1])
            ee_z_e = abs(EE_Cal_Pos[2]-EE_Act_Pos[2])
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
            self.ee_offset.append(ee_offset)
        else:
            rospy.logerr("End-Effector position are not of length 3")

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Start timer for the plan
        start_time = time()

        # Create an object for error analysis
        if ERR_ANALYSIS:
            errAnalysis = ErrorAnalysis(len(req.poses))

        ### Your FK code here
        # Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        a12 = 0.35     # meters from urdf file
        a23 = 1.25     # meters from urdf file
        a34 = -0.054   # meters from urdf file

        d01 = 0.75     # meters from urdf file
        d34 = 1.50     # meters from urdf file
        d67 = 0.303    # meters from urdf file

        # DH Parameters
        s = {alpha0:     0,  a0:   0, d1: d01, 
             alpha1: -pi/2,  a1: a12, d2:   0, q2: q2 -pi/2, 
             alpha2:     0,  a2: a23, d3:   0, 
             alpha3: -pi/2,  a3: a34, d4: d34,
             alpha4:  pi/2,  a4:   0, d5:   0,
             alpha5: -pi/2,  a5:   0, d6:   0,
             alpha6:     0,  a6:   0, d7: d67, q7: 0}

        # Homogeneous Transforms
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)

        T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_7 = T6_7.subs(s)

        # Transform from base link to end effector, required for error analysis
        if ERR_ANALYSIS:
          T0_7 = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)

        # Compute correctional rotation matrix
        R_corr_y = Matrix([[cos(-pi/2), 0, sin(-pi/2)],
                           [         0, 1,          0],
                           [-sin(-pi/2),0,cos(-pi/2)]]) 
        R_corr_z = Matrix([[cos(pi), -sin(pi), 0],
                           [sin(pi),  cos(pi), 0],
                           [      0,        0, 1]])
        R_corr = R_corr_z * R_corr_y

        # Compute inverse of rotation matrix from link 0 to link 3 by extracting 
        # rotational part from HT
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3_inv_g = R0_3.inv()

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
     
            # Compute rotational matrix between base link and gripper link
            # Correction needed to account of orientation difference between defintion 
            # of Grippper link in URDF versus DH Convention
            r, p, y = symbols('r p y')

            R_x = Matrix([[1, 0, 0],
                          [0,cos(r),-sin(r)],
                          [0,sin(r),cos(r)]]) 
            R_y = Matrix([[cos(p), 0, sin(p)],
                          [0,      1,   0],
                          [-sin(p),0,cos(p)]]) 
            R_z = Matrix([[cos(y), -sin(y), 0],
                          [sin(y),  cos(y), 0],
                          [     0,       0, 1]]) 

            R_rpy = (R_z.subs(y,yaw) *R_y.subs(p,pitch) * R_x.subs(r,roll) * R_corr)

            # Compute wrist center position using eq.(1)
            EE = Matrix([[px],
                         [py],
                         [pz]])
            WC = (EE - s[d7]* R_rpy[:,2])

            # Compute theta1 using eq.(2)
            theta1 = atan2(WC[1],WC[0])

            # Compute theta2 & theta3 using eq.(3)
            tc = sqrt(WC[0]**2 + WC[1]**2)
            alpha = atan2(WC[2] - d01, tc - a12)
            sb = sqrt((tc-a12)**2 + (WC[2]-d01)**2)

            angle_a = acos((a23**2 + sb**2 - (d34+0.001)**2)/(2*a23*sb))
            angle_b = acos((a23**2 + (d34+0.001)**2 - sb**2)/(2*a23*(d34+0.001)))

            theta2 =  pi/2 - angle_a - alpha
            theta3 =  pi/2 - (angle_b + 0.036)  # 0.036 is sag in link4 of -0.054m

            # Compute theta4, theta5 and theta6 using eq.(5) & eq.(6)
            R0_3_inv = R0_3_inv_g
            R0_3_inv = R0_3_inv.evalf(subs= {q1:theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3_inv * R_rpy

            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2),R3_6[1,2])

            # Non-singular case
            if sin(theta5) > 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            # Singular case
            else:
                theta4 = 0
                theta6 = atan2(-R3_6[0,1],R3_6[0,0])

            # Error analysis
            if ERR_ANALYSIS:
                # Find FK EE error
                FK = T0_7.evalf(subs={q1:theta1, q2: theta2, q3:theta3, q4:theta4,
                          q5:theta5, q6: theta6})

                # Compute RMSE
                # Populate RMSE and end-effector positions
                errAnalysis.root_mean_square_error([FK[0,3],FK[1,3], FK[2,3]],[px,py,pz])
                errAnalysis.append_ee_positions([FK[0,3],FK[1,3], FK[2,3]],[px,py,pz])
    
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        # Print error analysis report
        if ERR_ANALYSIS:
            errAnalysis.print_error_analysis_data()
        
        # Stop the timer for the plan
        rospy.loginfo("Total run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))
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
