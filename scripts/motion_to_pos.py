#!/usr/bin/env python3

# Example:
#
# 00:00:000,Pose1,0,0,-1,2.11,-1.18,0,0,0,-1,2.11,-1.18,0,-1.32,1.57,0,0,1.32,1.57,0,0
# 00:00:520,Pose2,0,0,0.48,1.47,0.106,0,0,0,0.48,1.47,0.106,0,-1.32,1.57,0,0,1.32,1.57,0,0
# 00:01:040,Pose3,0,0,0.48,1.47,0.106,0,0,0,0.48,1.47,0.106,0,-0.13,2.08,1.5,-0.2,0.13,2.08,-1.5,0.2
# 00:01:520,Pose4,0,0,-1.322,0.503,0.9,0,0,0,-1.322,0.503,0.9,0,-0.13,2.08,1.5,-0.2,0.13,2.08,-1.5,0.2
# 00:02:000,Pose4,0,0,-1.322,0.503,0.9,0,0,0,-1.322,0.503,0.9,0,-0.13,2.08,1.5,-0.2,0.13,2.08,-1.5,0.2
# 00:02:040,Pose5,-1,0,-1.77,1.8,0.7,0,-1,0,-1.3,0,0,0,-1.23,0,0.65,-0.2,0.11,1.75,-0.525,0.2
# 00:02:720,Pose5,-1,0,-1.77,1.8,0.7,0,-1,0,-1.3,0,0,0,-1.23,0,0.65,-0.2,0.11,1.75,-0.525,0.2
# 00:02:760,Pose6,-1,0,0.1,2,-1.18,0,-1,0.77,-1.29,0,0.9,0,0,0,1.52,-0.2,0.5,2.08,-0.34,0.2
# 00:04:000,Pose6,-1,0,0.1,2,-1.18,0,-1,0.77,-1.29,0,0.9,0,0,0,1.52,-0.2,0.5,2.08,-0.34,0.2
# 00:04:040,Pose7,-1,0,-0.3,2,-1.18,0,-1,0.77,-1.29,0,0.9,0,0,0,1.52,-0.2,0.5,2.08,0,0.2
# 00:04:280,Pose7,-1,0,-0.3,2,-1.18,0,-1,0.77,-1.29,0.5,0.9,0,0,0,1.52,-0.2,1,2.08,0,0.2
# 00:04:520,Pose8,0,0,-0.524,1.047,-0.524,0,0,0,-0.524,1.047,-0.524,0,0,0,0,0,0,0,0,0
# 00:04:800,Pose8,0,0,-0.524,1.047,-0.524,0,0,0,-0.524,1.047,-0.524,0,0,0,0,0,0,0,0,0
#

from math import degrees

dictio = {
    'HeadPitch': '0',
    'HeadYaw': '0',
    'LShoulderPitch': '0',
    'LShoulderRoll': '0',
    'LElbowYaw': '0',
    'LElbowRoll': '0',
    'LWristYaw': '0',
    'LHipYawPitch': '0',
    'LHipRoll': '0',
    'LHipPitch': '0',
    'LKneePitch': '0',
    'LAnklePitch': '0',
    'LAnkleRoll': '0',
    'RHipRoll': '0',
    'RHipPitch': '0',
    'RKneePitch': '0',
    'RAnklePitch': '0',
    'RAnkleRoll': '0',
    'RShoulderPitch': '0',
    'RShoulderRoll': '0',
    'RElbowYaw': '0',
    'RElbowRoll': '0',
    'RWristYaw': '0',
    'Lhand': '0',
    'RHand': '0'
}

file_output = open('output.pos', 'w')

with open('/home/vscode/ijnek_wrestle/controllers/motions/GetUpFront.motion', 'r') as motion_file:
    for line in motion_file:
        line = line.rstrip('\n').split(',')

        if line[0][0] == '#':
            joint_names = line[2:]
            continue

        time = line[0]
        label = line[1]
        joint_values = line[2:]

        for (name, val) in zip(joint_names, joint_values):
            dictio[name] = '%.2f' % degrees(float(val))

        (m, s, ms) = time.split(':')
        duration = int(s) * 1000 + int(ms)

        HeadYaw, HeadPitch, LElbowYaw, RElbowYaw, LWristYaw, RWristYaw, Lhand, RHand = ('0', '0', '0', '0', '0', '0', '0', '0')
        pos_line_list = ['!',
                         dictio['HeadPitch'],
                         dictio['HeadYaw'],
                         dictio['LShoulderPitch'],
                         dictio['LShoulderRoll'],
                         dictio['LElbowYaw'],
                         dictio['LElbowRoll'],
                         dictio['LWristYaw'],
                         dictio['LHipYawPitch'],
                         dictio['LHipRoll'],
                         dictio['LHipPitch'],
                         dictio['LKneePitch'],
                         dictio['LAnklePitch'],
                         dictio['LAnkleRoll'],
                         dictio['RHipRoll'],
                         dictio['RHipPitch'],
                         dictio['RKneePitch'],
                         dictio['RAnklePitch'],
                         dictio['RAnkleRoll'],
                         dictio['RShoulderPitch'],
                         dictio['RShoulderRoll'],
                         dictio['RElbowYaw'],
                         dictio['RElbowRoll'],
                         dictio['RWristYaw'],
                         dictio['Lhand'],
                         dictio['RHand'],
                         str(duration)
                        ]

        pos_line = ' '.join(pos_line_list) + '\n'
        file_output.write(pos_line)

file_output.close()
