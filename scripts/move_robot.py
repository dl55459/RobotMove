#!/usr/bin/env python3

from asyncio import PidfdChildWatcher
from matplotlib.transforms import LockableBbox
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time
import numpy as np
import sympy as sp
import math as math
from math import atan2, cos, sin, pi

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from my_robot_msgs.msg import JointAngles

    # xi(xi-1)
    # yi(yi-1)
    # zi(zi-1)

    # [xi(xi-1), xi(yi-1), xi(zi-1)]
    # [yi(xi-1), yi(yi-1), yi(zi-1)]
    # [zi(xi-1), zi(yi-1), zi(zi-1)]

    # L1z z-component of distance between base and motor 1
    # L3x x-component of distance between motor 1 and motor 3
    # L3z z-component of distance between motor 1 and motor 3
    # LEx x-component of distance between motor 3 and end effector
    # LEz z-component of distance between motor 3 and end effector

    # phi    angle of motor 1
    # theta  angle of motor 3

"""
^                                        8888b.  88 88""Yb 888888  dP""b8 888888
^                                         8I  Yb 88 88__dP 88__   dP   `"   88
^                                         8I  dY 88 88"Yb  88""   Yb        88
^                                        8888Y"  88 88  Yb 888888  YboodP   88
"""
# Function for calculating transformation matrix from base to end effector
def direct(L1z, L3x, L3z, LEx, LEz):
    phi, theta = sp.symbols("phi theta", real = True) # Angles

    # Translation matrix from base to motor 1
    BM1_T = sp.Matrix([
        [0],
        [0],
        [L1z] # 41.9
    ])

    # Rotation matrix from base to motor 1
    BM1_R = sp.Matrix([
        [sp.cos(phi), -sp.sin(phi), 0],
        [sp.sin(phi),  sp.cos(phi), 0],
        [          0,            0, 1]
    ])

    # Translation matrix from motor 1 to motor3
    M1M3_T = sp.Matrix([
        [L3x], # 190
        [0],
        [L3z] # -0.55
    ])

    # Rotation matrix from motor 1 to motor3
    M1M3_R = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0],
        [sp.sin(theta),  sp.cos(theta), 0],
        [          0,                0, 1]
    ])

    # Translation matrix from motor3 to end effector
    M3EE_T = sp.Matrix([
        [LEx], # 189
        [0],
        [LEz] # 39.35
    ])

    # Rotaion matrix from motor3 to end effector
    M3EE_R = sp.Matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Expanded matrix for homogeneous coordinates
    Expanded = sp.Matrix([
        [0, 0, 0, 1]
    ])

    # Create the transformation matrices for each segment
    TR = BM1_R.row_join(BM1_T)
    T_BM1 = TR.col_join(Expanded)
    TR = M1M3_R.row_join(M1M3_T)
    T_M1M3 = TR.col_join(Expanded)
    TR = M3EE_R.row_join(M3EE_T)
    T_M3EE = TR.col_join(Expanded)

    # Combine the transformation matrices to get the full transformation from base to end effector
    T_BEE = T_BM1 * T_M1M3 * T_M3EE # Calculate transformation matrix from base to end effector
    # T_BEE_subs = T_BEE.subs(subsL) # Substitute link lengths

    return T_BEE

"""
^                                        88 88b 88 Yb    dP 888888 88""Yb .dP"Y8 888888
^                                        88 88Yb88  Yb  dP  88__   88__dP `Ybo." 88__
^                                        88 88 Y88   YbdP   88""   88"Yb  o.`Y8b 88""
^                                        88 88  Y8    YP    888888 88  Yb 8bodP' 888888
"""
def inverse(LM1M3, LM3EE, xDesired, yDesired):

    r_sqr = xDesired**2 + yDesired**2
    cos_theta = (LM1M3**2 + LM3EE**2 - r_sqr) / (2 * LM1M3 * LM3EE)
    thetaDesired = sp.acos(cos_theta)

    beta  = math.atan2(yDesired, xDesired)
    gamma = math.atan2(LM3EE * sp.sin(thetaDesired),
                       LM1M3 + LM3EE * sp.cos(thetaDesired))
    phiDesired = beta - gamma
    return phiDesired, thetaDesired

def DEG2RAD(degree):

    rad = degree * (np.pi/180)
    return rad

def mm2m(mm):

    m = mm/1000
    return m

# Function sets default positions for active and in-active operation mode
def activePos(flag):
    # Check for activity flag
    if flag == 1:
        phiActive = DEG2RAD(0)
        alphaActive = DEG2RAD(0) # Active angle for motor 2 (pitch)
        thetaActive = DEG2RAD(0)
    else:
        phiActive = DEG2RAD(180) # Idle angle for motor 1
        alphaActive = DEG2RAD(-90) # Idle angle for motor 2
        thetaActive = DEG2RAD(165) # Idle angle for motor 3

    return phiActive, alphaActive, thetaActive

# When sending commands to Brot joint:
def BrotOffset(thetaD):

    offset = math.pi
    return (thetaD + offset) % (2 * math.pi)

# Distances for calculation
L1z = mm2m(41.9) # z-component of distance between base and motor 1
L3x = mm2m(190) # x-component of distance between motor 1 and motor 3
L3z = mm2m(-0.55) # z-component of distance between motor 1 and motor 3
LEx = mm2m(189) # x-component of distance between motor 3 and end effector
LEz = mm2m(39.35) # z-component of distance between motor 3 and end effector

LM1M3 = mm2m(190) # x-component of distance between motor 1 and motor 3
LM3EE = mm2m(189) # x-component of distance between motor 3 and end effector

# Working area limits
# WS = Workspace
# B = Base
xLowerWS = mm2m(0) # -25.487)
xUpperWS = mm2m(350) # 324.513)
xLowerB = mm2m(100) # 25.487)
xUpperB = mm2m(275) # 124.513)
yLowerWS = mm2m(0) # -175)
yUpperWS = mm2m(350) # 175)
yLowerB = mm2m(200) # -75)
yUpperB = mm2m(350) # 75)

def R2B(xR, yR):

    xB = -xR + 324.513
    yB = -yR + 175
    return xB, yB

# Global variables
finishedFlag = False
Brot = 0
Pitch = 0
EErot = 0

class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # Define publishers and subscribers
        self.robot_goal_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ready_pub = self.create_publisher(Bool, '/pathfinder/ready', 10)
        self.auto_sub = self.create_subscription(Path, '/pathfinder/path', self.autoMove, 10)
        self.manualAngles_sub = self.create_subscription(JointAngles, '/manual_angles', self.manualMoveAngles, 10)
        self.manualPoints_sub = self.create_subscription(Point, '/manual/xy', self.manualMovePoints, 10)
# -----------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------
    def autoMove(self, data):

        self.ready_pub.publish(Bool(data = False))

        Brot, Pitch, EErot = activePos(False) # Set arm to idle position (All positions taken to account)
        self.move_robot([Brot, Pitch, EErot], durationS = 1.5)

        for i, coordinate in enumerate(data.poses):

            x = coordinate.pose.position.x
            y = coordinate.pose.position.y
            self.get_logger().info(f"Processing waypoint at x: {x:.3f}, y: {y:.3f}")

            if (xLowerWS <= x <= xUpperWS) and (yLowerWS <= y <= yUpperWS):
                self.get_logger().info(f"Work area")

                if (xLowerB <= x <= xUpperB) and (yLowerB <= y <= yUpperB):
                    self.get_logger().info("Inaccessible area")

                else:
                    x, y = R2B(x, y)

                    Brot, EErot = inverse(LM1M3, LM3EE, x, y)
                    self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)

                    if i == 1: # After first coordinate loaded set arm to active position
                        _, Pitch, _ = activePos(True)
                        self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)
            else:
                self.get_logger().info("Outside of work area")

        Brot, Pitch, EErot = activePos(False) # Set arm to idle position (All positions taken to account)
        self.move_robot([Brot, Pitch, EErot], durationS = 3)

        self.ready_pub.publish(Bool(data = True))
        self.get_logger().info("Path execution completed")

    def manualMoveAngles(self, data):

        try:
            # Get phi, alpha, theta angles
            phi = float(data.phi)
            alpha = float(data.alpha)
            theta = float(data.theta)

            # Convert to rad
            phi_rad = DEG2RAD(phi)
            alpha_rad = DEG2RAD(alpha)
            theta_rad = DEG2RAD(theta)

            # Move robot
            self.move_robot([phi_rad, alpha_rad, BrotOffset(theta_rad)], durationS = 1.5)

        except Exception as e:
            self.get_logger().error(f"Error processing manual move: {str(e)}")

    def manualMovePoints(self, data):

        try:
            # Get x, y coordinates
            x = float(data.x)
            y = float(data.y)
            # Set motora angles according to coordinates
            Brot, EErot = inverse(LM1M3, LM3EE, x, y)
            _, Pitch, _ = activePos(False)
            self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)
            _, Pitch, _ = activePos(True)
            self.move_robot([Brot, Pitch, BrotOffset(EErot)], durationS = 1.5)

        except Exception as e:
            self.get_logger().error(f"Error processing manual move: {str(e)}")

    def move_robot(self, q, durationS):

        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('joint1')
        goal_trajectory.joint_names.append('joint2')
        goal_trajectory.joint_names.append('joint3')

        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(q[0])
        goal_point.positions.append(q[1])
        goal_point.positions.append(q[2])
        goal_point.time_from_start = Duration(durationS).to_msg()

        goal_trajectory.points.append(goal_point)

        return self.robot_goal_publisher_.publish(goal_trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()