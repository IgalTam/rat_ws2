#!/usr/bin/python3
from email import parser
from ntpath import join
import sys
import copy
from turtle import forward

import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import argparse as ap
from math import pi, tau, fabs, cos
from kinematics import inverseKinematics, forwardKinematics
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from roboclaw_python.roboclaw_3 import Roboclaw # for actuating claw, as rollmotor is NOT in urdf and not visible to moveit



"""
pose.position:
    y value of arm is fixed at -0.09432, 
    x value increases as claw gets further from base/rover
    z value inreases as claw gains elevation in relation to base 
"""

# def inverseKinematics(x:float, z:float, phi_lo:int, phi_hi:int) -> tuple:
#     # Length of links in cm
#     l1 = BASE_TO_ELBOW; l2 = ELBOW_TO_WRIST; l3 = WRIST_TO_EEF

#     # Desired Position of End effector
#     px = x
#     pz = z

#     #phi = deg2rad(phi)
#     # Equations for Inverse kinematics
#     with warnings.catch_warnings():
#         warnings.simplefilter('ignore') 
#         for phi in range(phi_lo, phi_hi+1):
#             phi = np.deg2rad(phi)
#             wx = px - l3*cos(phi)
#             wz = pz - l3*np.sin(phi)

#             # delta = wx**2 + wz**2
#             # c2 = ( delta -l1**2 -l2**2)/(2*l1*l2)
#             # s2 = np.sqrt(1-c2**2)  # elbow down
#             # theta_2 = np.arctan2(s2, c2)

#             theta_2 = pi - np.arccos((l1**2 + l2**2 - wx**2 - wz**2)/(2*l1*l2))
#             theta_1 = np.arctan2(wz,wx) \
#                 - np.arccos((wx**2 + wz**2 + l1**2 - l2**2)/(2*l1*np.sqrt(wx**2 + wz**2)))
#             # s1 = ((l1+l2*c2)*wz - l2*s2*wx)/delta
#             # c1 = ((l1+l2*c2)*wx + l2*s2*wz)/delta
#             # theta_1 = np.arctan2(s1,c1)
#             theta_3 = phi-theta_1-theta_2

#             gamma = np.arctan2(wz,wx) - theta_1
#             theta_1_prime = theta_1 + 2*gamma
#             theta_2_prime = -theta_2
#             theta_3_prime = phi - theta_1_prime - theta_2_prime
#             def check_joint_limits(t1, t2, t3):
#                 if t1 > 0 or t1 < -3.2:
#                     return False
#                 elif t2 > 0 or t2 < -3.2:
#                     return False
#                 elif t3 > 3.2 or t3 < -3.2:
#                     return False
#                 return True
#             theta_1_m_p = theta_1_prime - pi ; theta_2_m_p = -(theta_2_prime + pi); theta_3_m_p = theta_3_prime + pi/2
#             theta_1_m = theta_1 - pi ; theta_2_m = -(theta_2 + pi); theta_3_m = theta_3+ pi/2

#             #if(not np.isnan(theta_1) and check_joint_limits(theta_1_m_p, theta_2_m_p, theta_3_m_p)):
#             #print(phi)
#             if(not np.isnan(theta_1_prime)):
#                 # print('theta_1: ', np.rad2deg(theta_1), theta_1)
#                 # print('theta_2: ', np.rad2deg(theta_2), theta_2)
#                 # print('theta_3: ', np.rad2deg(theta_3), theta_3)
#                 # theta_1 = theta_1 - pi; theta_2 =  -(theta_2 - pi); theta_3 = theta_3 + pi/2
#                 # print('theta_mapped1: ', np.rad2deg(theta_1), theta_1)
#                 # print('theta_mapped2: ', np.rad2deg(theta_2), theta_2)
#                 # print('theta_mapped3: ', np.rad2deg(theta_3), theta_3)
#                 # print('theta_1_m:  ', np.rad2deg(theta_1_m), theta_1_m)
#                 # print('theta_2_m:  ', np.rad2deg(theta_2_m), theta_2_m)
#                 # print('theta_3_m:  ', np.rad2deg(theta_3_m), theta_3_m)

#                 # print('\ntheta_prime_m1: ', np.rad2deg(theta_1_m_p), theta_1_m_p)
#                 # print('theta_prime_m2: ', np.rad2deg(theta_2_m_p), theta_2_m_p)
#                 # print('theta_prime_m3: ', np.rad2deg(theta_3_m_p), theta_3_m_p)
#                 while theta_3_m_p > pi:
#                     theta_3_m_p -= 2*pi
#                 while theta_1_m > pi:
#                     theta_1_m -= 2*pi
#                 while theta_1_m < -pi:
#                     theta_1_m += 2*pi

#                 while theta_2_m > pi:
#                     theta_2_m -= 2*pi
#                 while theta_2_m < -pi:
#                     theta_2_m += 2*pi
#                 while theta_3_m > pi:
#                     theta_3_m -= 2*pi
#                 while theta_3_m < -pi:
#                     theta_3_m += 2*pi

#                 if(check_joint_limits(theta_1_m_p, theta_2_m_p, theta_3_m_p)):
#                     return (theta_1_m_p, theta_2_m_p, theta_3_m_p), phi
#                 elif(check_joint_limits(theta_1_m, theta_2_m, theta_3_m)):
#                     return (theta_1_m, theta_2_m, theta_3_m), phi
#     return None

# def forwardKinematics(joint_vals: list, silent=False) -> tuple:
#     # code here :
#     #https://github.com/aakieu/3-dof-planar/blob/master/DHFowardKinematics.py
    
#     # dh parameters explained here:
#     #https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps

#     # the distance between the previous x-axis and the current x-axis, along the previous z-axis.
#     d1 = 0; d2 = 0; d3 = 0 # not actually zero from elbow to wrist

#     # Angles in radians between links, each joint will need some remapping
#     theta_1 = joint_vals[0] + pi
#     theta_2 = -joint_vals[1] + pi
#     theta_3 = joint_vals[2] - pi/2
#     #theta_1 = 3.1415 ; theta_2 = 0 ; theta_3 = 0
#     #print(f't1 {np.rad2deg(theta_1)} t2 {np.rad2deg(theta_2)} t3 {np.rad2deg(theta_3)}')
#     # length of common normal, distance between prev z axis and cur z axis, along x axis
#     r1 = BASE_TO_ELBOW; r2 = ELBOW_TO_WRIST; r3 = WRIST_TO_EEF

#     # angle around common nomral between prev z and cur z
#     alphl1 = 0; alphl2 = 0; alphl3 = 0

#     # DH Parameter Table for 3 DOF Planar
#     PT = [[theta_1, alphl1, r1, d1],
#           [theta_2, alphl2, r2, d2],
#           [theta_3, alphl3, r3, d3]]

#     # Homogeneous Transformation Matrices
#     i = 0
#     H0_1 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
#             [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
#             [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
#             [0, 0, 0, 1]]

#     i = 1
#     H1_2 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
#             [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
#             [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
#             [0, 0, 0, 1]]

#     i = 2
#     H2_3 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
#             [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
#             [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
#             [0, 0, 0, 1]]

#     # print("H0_1 =")
#     # print(np.matrix(H0_1))
#     # print("H1_2 =")
#     # print(np.matrix(H1_2))
#     # print("H2_3 =")
#     # print(np.matrix(H2_3))

#     H0_2 = np.dot(H0_1,H1_2)
#     H0_3 = np.dot(H0_2,H2_3)
#     if not silent:
#         print("H0_3 =")
#         print(np.matrix(H0_3))
#     return H0_3[0][3], H0_3[1][3] # x, z coords of eef

class MoveGroupInterface(object):
    def __init__(self, silent=False):
        # super()
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=True)


        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "rat_arm" # defined in setup assistant
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        if(not silent):
            # We can get the name of the reference frame for this robot:
            
            print("============ Planning frame: %s" % planning_frame)

            # We can also print the name of the end-effector link for this group:
            print("============ End effector link: %s" % eef_link)

            # We can get a list of all the groups in the robot:
            print("============ Available Planning Groups:", robot.get_group_names())

            # Sometimes for debugging it is useful to print the entire state of the
            # robot:
            print("============ Printing robot state")
            print(robot.get_current_state())
            print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def actuate_claw(self):
        rc = Roboclaw("/dev/ttyS0", 115200)
        rc.Open()
        address = 0x81
        rc.SetEncM1(address, 0) # reset this encoder
        rc.SpeedAccelDeccelPositionM1(address,0,100,0,57,1)

    def go_to_joint_goal(self, joint_angles:list):

        self.move_group.set_joint_value_target({"base_joint": joint_angles[0],
                                                 "elbow_joint":joint_angles[1],
                                                 "wrist_joint":joint_angles[2]})
        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

    def get_cur_pose(self):
        pos = self.move_group.get_current_pose().pose.position
        return pos.x, pos.z, 0, 360

    def print_cur_pose(self):
        print(f'Current Pose: \n{self.move_group.get_current_pose().pose.position}\n')

    def print_joint_state(self):
        joint_state = self.move_group.get_current_joint_values()
        print(f'Current Joint Values: {joint_state}')
    
    def named_move(self, named_pose):
        move_group = self.move_group
        move_group.set_named_target(named_pose)
        move_group.go(wait=True)
        move_group.stop()

def main_interactive():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the RAT Interactive Mode")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("\n Usage:\n  x, z and phi prompts will occur on screen once started. Enter distance in meters (float) for x and z, and degrees (integer) for phi.")
        print("  To keep the current x, z, or phi value type \'X\' or non numeric character\n")
        input("============ Press `Enter` to begin connect to Robot ==============")
        interface = MoveGroupInterface(silent=True)
        # get inital x, z, phi values
        x, z= forwardKinematics(interface.move_group.get_current_joint_values(), silent=True) 
        phi_lo = 310
        phi_hi = 311
        print(f'x: {x:0.5f} z: {z:0.5f} phi range: {phi_lo} - {phi_hi}\n')
        while(1):
            #input("============ Press `Enter` to print current pose ============")
            
            claw = input("open/close claw? (yes/n): ")
            if claw == "yes":
                interface.actuate_claw()
                continue
            new_x = input("x: ")
            new_z = input("z: ")
            new_phi_lo = input("phi low: ")
            new_phi_hi = input("phi hi: ")


            # check if input is int or float
            if new_x.replace('.', '', 1).isdigit(): 
                x = float(new_x)
            if new_z.replace('.', '', 1).isdigit(): 
                z = float(new_z)
            if new_phi_lo.replace('.', '', 1).isdigit(): 
                new_phi_lo = int(new_phi_lo)
                if new_phi_lo >= 0 and new_phi_lo<= 360: # check its a valid angle
                    phi_lo = int(new_phi_lo)
            if new_phi_hi.replace('.', '', 1).isdigit(): 
                new_phi_hi = int(new_phi_hi)
                if new_phi_hi >= 0 and new_phi_hi <= 360:
                    phi_hi = int(new_phi_hi)
            print(f'x: {x:0.5f} z: {z:0.5f} phi range: {phi_lo} - {phi_hi}')
            if input("Search for Joint Solution? (y/n): ") == 'y':
                sol = inverseKinematics(x, z, phi_lo=phi_lo, phi_hi=phi_hi)
                if not sol:
                    print(f'No solution found...')
                else:
                    joint_solution_angles, phi = sol
                    print(f'\nSolution Found with phi == {np.rad2deg(phi):0.0f}:\n{joint_solution_angles}\nSending to Moveit...')
                    interface.go_to_joint_goal(joint_solution_angles)

    except rospy.exceptions.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

def main_cmd(x=None, z=None, phi_range=None, claw=None, fk=None):
    print(f'x:{x}, z:{z}, claw:{claw}')
    # if not(x and z):
    #     print("Must provide BOTH X and Z values!")
    #     return
    print("Connecting to Moveit...")
    arm_interface = MoveGroupInterface(silent=True)
    if claw:
        arm_interface.actuate_claw()
    if phi_range and len(phi_range) == 1: # make list have two elements, that are the same
        phi_range.append(phi_range[0])
    if fk:
        cur_joint_vals = arm_interface.move_group.get_current_joint_values()
        forwardKinematics(cur_joint_vals)
    else:
        joint_solution_angles, _ = inverseKinematics(x, z, phi_lo=phi_range[0], phi_hi=phi_range[1])
        print(arm_interface.move_group.get_current_joint_values())
        if not joint_solution_angles:
            print("No solution found exiting...")
            return
        arm_interface.go_to_joint_goal(joint_solution_angles)


    
if __name__ == "__main__":
    parser = ap.ArgumentParser()
    subparsers = parser.add_subparsers(dest='subcommand')
    subparsers.required = True
    parser_fk = subparsers.add_parser('fwd_kin')
    parser_ik = subparsers.add_parser('inv_kin')
    parser_int = subparsers.add_parser('interactive')
    parser_ik.add_argument('-x', type=float, required=True, help='X coordinate in meters')
    parser_ik.add_argument('-z', type=float, required=True, help='Z coordinate in meters')
    parser_ik.add_argument('--phi_range', type=int, nargs=2, required=True, help='range of angles for end effector orientation (degrees), angle is relative to x-axis')
    parser_ik.add_argument('--angle', type=float, required=False, help='Angle of end effector relative to x-axis')

    parser_ik.add_argument('--claw', action='store_true', required=False, help='print forward kinematics matrix of current joint values')
    #parser_ik.add_argument('--interactive', action='store_true', required=False, help='run program in interactive mode')
    #print(parser.parse_args())

    args = parser.parse_args()
    if args.subcommand == 'fwd_kin':
        main_cmd(fk=True)
    elif args.subcommand == 'interactive':
        main_interactive()
    elif not [x for x in vars(args).values() if x]: # check if any args were passed
        parser.print_help()
    else:
        main_cmd(args.x, args.z, args.phi_range, args.claw)
        
