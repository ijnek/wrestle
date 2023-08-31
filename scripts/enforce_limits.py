#!/usr/bin/env python3

from math import degrees, radians

class JointLimit(object):
    def __init__(self, name, lower, upper):
        self.name = name
        self.lower = lower
        self.upper = upper

joint_limits = [
    JointLimit('HeadYaw', -2.08567, 2.08567),
    JointLimit('HeadPitch', -0.671952, 0.514872),
    JointLimit('LShoulderPitch', -2.08567, 2.08567),
    JointLimit('LShoulderRoll', -0.314159, 1.32645),
    JointLimit('LElbowYaw', -2.08567, 2.08567),
    JointLimit('LElbowRoll', -1.54462, 0),  # Max not in proto file
    JointLimit('LWristYaw', -3.14, 3.14),  # Min, Max not in proto file
    JointLimit('LHipYawPitch', -1.14529, 0.740718),
    JointLimit('LHipRoll', -0.379435, 0.79046),
    JointLimit('LHipPitch', -1.77378, 0.48398),
    JointLimit('LKneePitch', -0.0923279, 2.11255),
    JointLimit('LAnklePitch', -1.18944, 0.922581),
    JointLimit('LAnkleRoll', -0.397880, 0.769001),
    JointLimit('RHipRoll', -0.738274, 0.449597),
    JointLimit('RHipPitch', -1.77378, 0.48398),
    JointLimit('RKneePitch', -0.0923279, 2.11255),
    JointLimit('RAnklePitch', -1.1863, 0.932006),
    JointLimit('RAnkleRoll', -0.768992, 0.397935),
    JointLimit('RShoulderPitch', -2.08567, 2.08567),
    JointLimit('RShoulderRoll', -1.32645, 0.314159),
    JointLimit('RElbowYaw', -2.08567, 2.08567),
    JointLimit('RElbowRoll', 0, 1.54462),  # Min not in proto file
    JointLimit('RWristYaw', -3.14, 3.14),  # Min, Max not in proto file
    JointLimit('LHand', -3.14, 3.14),  # Min, Max not in proto file
    JointLimit('RHand', -3.14, 3.14)  # Min, Max not in proto file
]

while True:
    line = input("joint: ")
    line = [radians(int(joint_position))for joint_position in line.split()]
    for i in range(25):
        if line[i] < joint_limits[i].lower:
            line[i] = joint_limits[i].lower
        elif line[i] > joint_limits[i].upper:
            line[i] = joint_limits[i].upper
    print(' '.join([str(degrees(joint_position)) for joint_position in line]))
