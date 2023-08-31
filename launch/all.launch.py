# Copyright 2023 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nao_lola_client_node = Node(package='nao_lola_client', executable='nao_lola_client')

    ik_node = Node(package='nao_ik', executable='ik_node')

    nao_phase_provider_node = Node(package='nao_phase_provider', executable='nao_phase_provider',
                                   remappings=[('fsr', 'sensors/fsr')])

    walk_node = Node(package='walk', executable='walk', remappings=[('imu', 'sensors/imu')])

    nao_lola_conversion_node = Node(package='nao_lola_conversion', executable='nao_lola_conversion')

    lower_arms = ExecuteProcess(
        cmd=['ros2 topic pub --once /effectors/joint_positions nao_lola_command_msgs/msg/JointPositions "{indexes: [2, 18], positions: [1.517, 1.517]}"'],
        shell=True)

    getup_back_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'getupBack.pos'])
    getup_back_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                     parameters=[{'file': getup_back_pos_path}],
                     remappings=[('start_pos_action', 'start_getup_back')])

    getup_front_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'getupFront.pos'])
    getup_front_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                     parameters=[{'file': getup_front_pos_path}],
                     remappings=[('start_pos_action', 'start_getup_front')])

    lean_forward_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'leanForward.pos'])
    lean_forward_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                     parameters=[{'file': lean_forward_pos_path}],
                     remappings=[('start_pos_action', 'start_lean_forward')])

    # twist_forward = ExecuteProcess(
    #     cmd=['ros2 topic pub --once /target geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'],
    #     shell=True)

    motion_manager_node = Node(package='wrestle', executable='motion_manager',
                               remappings=[('imu', 'sensors/filtered_imu')])

    complementary_filter_node = Node(package='imu_complementary_filter',
                                     executable='complementary_filter_node',
                                     remappings=[('imu/data_raw', 'sensors/imu'),
                                                 ('imu/data', 'sensors/filtered_imu')],
                                     parameters=[{'publish_tf': True}])

    return LaunchDescription([
        nao_lola_client_node,
        # ik_node,
        # nao_phase_provider_node,
        # walk_node,
        nao_lola_conversion_node,
        # lower_arms,
        getup_back_node,
        getup_front_node,
        lean_forward_node,
        # twist_forward,
        motion_manager_node,
        complementary_filter_node
    ])
