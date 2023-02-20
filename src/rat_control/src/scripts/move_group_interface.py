#!/usr/bin/python3
import sys


import rospy
from roboclaw_driver.msg import armCmd
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
        self.cmd_pub = rospy.Publisher('roboclaw_cmd', armCmd, queue_size=10) # for publishing claw actuation
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def actuate_claw(self):
        # rc = Roboclaw("/dev/ttyS0", 115200)
        # rc.Open()
        # address = 0x81
        # rc.SetEncM1(address, 0) # reset this encoder
        # rc.SpeedAccelDeccelPositionM1(address,0,100,0,57,1)
        msg = armCmd()
        msg.position_rads = [0, 0, 0, -1]
        msg.speed = [0, 0, 0, 0]
        msg.accel_deccel = [0, 0, 0, 0]
        self.cmd_pub.publish(msg)
    
    def position_claw(self, angle_rad: int):
        """rotate claw motor to input angle"""
        msg = armCmd()
        msg.position_rads = [0, 0, 0, angle_rad]
        msg.speed = [0, 0, 0, 0]
        msg.accel_deccel = [0, 0, 0, 0]
        self.cmd_pub.publish(msg)
        # UNFINISHED
        
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
            #phi_lo = 270
            #phi_hi = 280

            # check if input is int or float

            if new_x and new_x[0] == '-' and new_x[1:].replace('.', '', 1).isdigit(): # used to handle negative values
                x = float(new_x)
            elif new_x.replace('.', '', 1).isdigit(): 
                x = float(new_x)

            if new_z and new_z[0] == '-' and new_z[1:].replace('.', '', 1).isdigit(): # used to handle negative values
                z = float(new_z)
            elif new_z.replace('.', '', 1).isdigit(): 
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
    #     print("Must provide BOTH X and Z values!") (phi_low, phi_hi) phi_low < phi < phi_hi
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


def nuada_demo():
    """demo function for DBT1"""
    pass
    

if __name__ == "__main__":
    parser = ap.ArgumentParser()
    subparsers = parser.add_subparsers(dest='subcommand')
    subparsers.required = True
    parser_fk = subparsers.add_parser('fwd_kin')
    parser_ik = subparsers.add_parser('inv_kin')
    parser_int = subparsers.add_parser('interactive')
    parser_nu = subparsers.add_parser('nuada_main')

    parser_ik.add_argument('-x', type=float, required=True, help='X coordinate in meters')
    parser_ik.add_argument('-z', type=float, required=True, help='Z coordinate in meters')
    parser_ik.add_argument('--phi_range', type=int, nargs=2, required=True, help='range of angles for end effector orientation (degrees), angle is relative to x-axis')
    parser_ik.add_argument('--angle', type=float, required=False, help='Angle of end effector relative to x-axis')
    parser_ik.add_argument('--claw', action='store_true', required=False, help='print forward kinematics matrix of current joint values')

    parser_nu.add_argument('-x', type=float, required=True, help='X coordinate in meters')
    parser_nu.add_argument('-z', type=float, required=True, help='Z coordinate in meters')
    parser_nu.add_argument('--phi_range', type=int, nargs=2, required=True, help='range of angles for end effector orientation (degrees), angle is relative to x-axis')
    parser_nu.add_argument('--claw', type=float, required=False, help='Angle of end effector relative to x-axis')
    
    #parser_ik.add_argument('--interactive', action='store_true', required=False, help='run program in interactive mode')
    #print(parser.parse_args())

    args = parser.parse_args()
    if args.subcommand == 'fwd_kin':
        main_cmd(fk=True)
    elif args.subcommand == 'interactive':
        main_interactive()
    elif args.subcommand == 'nuada_main':
        nuada_main_cmd(args.x, args.z, args.phi_range, args.claw)
    elif not [x for x in vars(args).values() if x]: # check if any args were passed
        parser.print_help()
    else:
        main_cmd(args.x, args.z, args.phi_range, args.claw)
        
