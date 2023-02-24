import numpy as np
from math import pi, tau, fabs, cos

# # link lengths in meters
# BASE_TO_ELBOW = 0.235 
# ELBOW_TO_WRIST = 0.2695
# WRIST_TO_EEF = 0.193
# import warnings

# link lengths in centimeters
BASE_TO_ELBOW = 23.5 
ELBOW_TO_WRIST = 26.95
WRIST_TO_EEF = 19.3
import warnings

def inverseKinematics(x:float, z:float, phi_lo:int, phi_hi:int) -> tuple:
    # Length of links in cm
    l1 = BASE_TO_ELBOW; l2 = ELBOW_TO_WRIST; l3 = WRIST_TO_EEF

    # Desired Position of End effector
    px = x
    pz = z

    #phi = deg2rad(phi)
    # Equations for Inverse kinematics
    with warnings.catch_warnings():
        warnings.simplefilter('ignore') 
        for phi in range(phi_lo, phi_hi+1):
            phi = np.deg2rad(phi)
            wx = px - l3*cos(phi)
            wz = pz - l3*np.sin(phi)

            # delta = wx**2 + wz**2
            # c2 = ( delta -l1**2 -l2**2)/(2*l1*l2)
            # s2 = np.sqrt(1-c2**2)  # elbow down
            # theta_2 = np.arctan2(s2, c2)

            theta_2 = pi - np.arccos((l1**2 + l2**2 - wx**2 - wz**2)/(2*l1*l2))
            theta_1 = np.arctan2(wz,wx) \
                - np.arccos((wx**2 + wz**2 + l1**2 - l2**2)/(2*l1*np.sqrt(wx**2 + wz**2)))
            # s1 = ((l1+l2*c2)*wz - l2*s2*wx)/delta
            # c1 = ((l1+l2*c2)*wx + l2*s2*wz)/delta
            # theta_1 = np.arctan2(s1,c1)
            theta_3 = phi-theta_1-theta_2

            gamma = np.arctan2(wz,wx) - theta_1
            theta_1_prime = theta_1 + 2*gamma
            theta_2_prime = -theta_2
            theta_3_prime = phi - theta_1_prime - theta_2_prime
            def check_joint_limits(t1, t2, t3):
                if t1 > 0 or t1 < -3.2:
                    return False
                elif t2 > 0 or t2 < -3.2:
                    return False
                elif t3 > 3.2 or t3 < -3.2:
                    return False
                return True
            theta_1_m_p = theta_1_prime - pi ; theta_2_m_p = -(theta_2_prime + pi); theta_3_m_p = theta_3_prime + pi/2
            theta_1_m = theta_1 - pi ; theta_2_m = -(theta_2 + pi); theta_3_m = theta_3+ pi/2

            #if(not np.isnan(theta_1) and check_joint_limits(theta_1_m_p, theta_2_m_p, theta_3_m_p)):
            #print(phi)
            if(not np.isnan(theta_1_prime)):
                # print('theta_1: ', np.rad2deg(theta_1), theta_1)
                # print('theta_2: ', np.rad2deg(theta_2), theta_2)
                # print('theta_3: ', np.rad2deg(theta_3), theta_3)
                # theta_1 = theta_1 - pi; theta_2 =  -(theta_2 - pi); theta_3 = theta_3 + pi/2
                # print('theta_mapped1: ', np.rad2deg(theta_1), theta_1)
                # print('theta_mapped2: ', np.rad2deg(theta_2), theta_2)
                # print('theta_mapped3: ', np.rad2deg(theta_3), theta_3)
                # print('theta_1_m:  ', np.rad2deg(theta_1_m), theta_1_m)
                # print('theta_2_m:  ', np.rad2deg(theta_2_m), theta_2_m)
                # print('theta_3_m:  ', np.rad2deg(theta_3_m), theta_3_m)

                # print('\ntheta_prime_m1: ', np.rad2deg(theta_1_m_p), theta_1_m_p)
                # print('theta_prime_m2: ', np.rad2deg(theta_2_m_p), theta_2_m_p)
                # print('theta_prime_m3: ', np.rad2deg(theta_3_m_p), theta_3_m_p)
                while theta_3_m_p > pi:
                    theta_3_m_p -= 2*pi
                while theta_1_m > pi:
                    theta_1_m -= 2*pi
                while theta_1_m < -pi:
                    theta_1_m += 2*pi

                while theta_2_m > pi:
                    theta_2_m -= 2*pi
                while theta_2_m < -pi:
                    theta_2_m += 2*pi

                while theta_3_m > pi:
                    theta_3_m -= 2*pi
                while theta_3_m < -pi:
                    theta_3_m += 2*pi

                if(check_joint_limits(theta_1_m_p, theta_2_m_p, theta_3_m_p)):
                    return (theta_1_m_p, theta_2_m_p, theta_3_m_p), phi
                elif(check_joint_limits(theta_1_m, theta_2_m, theta_3_m)):
                    return (theta_1_m, theta_2_m, theta_3_m), phi
    return None

def forwardKinematics(joint_vals: list, silent=False) -> tuple:
    # code here :
    #https://github.com/aakieu/3-dof-planar/blob/master/DHFowardKinematics.py
    
    # dh parameters explained here:
    #https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps

    # the distance between the previous x-axis and the current x-axis, along the previous z-axis.
    d1 = 0; d2 = 0; d3 = 0 # not actually zero from elbow to wrist

    # Angles in radians between links, each joint will need some remapping
    theta_1 = joint_vals[0] + pi
    theta_2 = -joint_vals[1] + pi
    theta_3 = joint_vals[2] - pi/2
    #theta_1 = 3.1415 ; theta_2 = 0 ; theta_3 = 0
    #print(f't1 {np.rad2deg(theta_1)} t2 {np.rad2deg(theta_2)} t3 {np.rad2deg(theta_3)}')
    # length of common normal, distance between prev z axis and cur z axis, along x axis
    r1 = BASE_TO_ELBOW; r2 = ELBOW_TO_WRIST; r3 = WRIST_TO_EEF

    # angle around common nomral between prev z and cur z
    alphl1 = 0; alphl2 = 0; alphl3 = 0

    # DH Parameter Table for 3 DOF Planar
    PT = [[theta_1, alphl1, r1, d1],
          [theta_2, alphl2, r2, d2],
          [theta_3, alphl3, r3, d3]]

    # Homogeneous Transformation Matrices
    i = 0
    H0_1 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
            [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    i = 1
    H1_2 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
            [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    i = 2
    H2_3 = [[cos(PT[i][0]), -np.sin(PT[i][0])*cos(PT[i][1]), np.sin(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
            [np.sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*np.sin(PT[i][1]), PT[i][2]*np.sin(PT[i][0])],
            [0, np.sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

    # print("H0_1 =")
    # print(np.matrix(H0_1))
    # print("H1_2 =")
    # print(np.matrix(H1_2))
    # print("H2_3 =")
    # print(np.matrix(H2_3))

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    if not silent:
        print("H0_3 =")
        print(np.matrix(H0_3))
    return H0_3[0][3], H0_3[1][3] # x, z coords of eef
