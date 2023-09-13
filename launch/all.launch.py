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

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    webots_controller_urdf_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'urdf', 'webots_controller.urdf'])
    webots_controller = WebotsController(
        robot_name=os.environ.get('WEBOTS_CONTROLLER_URL'),
        parameters=[{'robot_description': webots_controller_urdf_path}])

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
                           name='getup_back', parameters=[{'file': getup_back_pos_path}],
                           remappings=[('start_pos_action', 'start_getup_back')])

    getup_front_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'getupFront.pos'])
    getup_front_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                            name='getup_front', parameters=[{'file': getup_front_pos_path}],
                            remappings=[('start_pos_action', 'start_getup_front')])

    lean_forward_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'leanForward.pos'])
    lean_forward_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                             name='lean_forward', parameters=[{'file': lean_forward_pos_path}],
                             remappings=[('start_pos_action', 'start_lean_forward')])

    # twist_forward = ExecuteProcess(
    #     cmd=['ros2 topic pub --once /target geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'],
    #     shell=True)

    motion_manager_node = Node(package='wrestle', executable='motion_manager',
                               remappings=[('imu', 'sensors/filtered_imu'),
                                           ('sonar', 'sensors/sonar'),
                                           ('twist', 'target'),
                                           ('walk_change_state', 'Walk/change_state')])

    imu_filter_madgwick_node = Node(package='imu_filter_madgwick',
                                    executable='imu_filter_madgwick_node',
                                    remappings=[('imu/data_raw', 'sensors/imu'),
                                                ('imu/data', 'sensors/filtered_imu')],
                                    parameters=[{'publish_tf': True},
                                                {'reverse_tf': True},
                                                {'use_mag': False},
                                                {'gain': 0.3},
                                                ])

    nao_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nao_state_publisher'), 'launch', 'nao_state_publisher_launch.py']
            )
        )
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'rviz', 'default.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    webots_nao_camera_top = Node(package='webots_nao_camera', executable='webots_nao_camera',
                                 parameters=[{'port': 10001}],
                                 remappings=[('image', 'image_top')])
    webots_nao_camera_bot = Node(package='webots_nao_camera', executable='webots_nao_camera',
                                 parameters=[{'port': 10002}],
                                 remappings=[('image', 'image_bot')])

    send_goal_walk = ExecuteProcess(
        cmd=['ros2 action send_goal /walk walk_interfaces/action/Walk "{twist: {linear: {x: 0.1}}}"'],
        shell=True)

    return LaunchDescription([
        webots_controller,
        nao_lola_client_node,
        ik_node,
        nao_phase_provider_node,
        walk_node,
        nao_lola_conversion_node,
        lower_arms,
        getup_back_node,
        getup_front_node,
        lean_forward_node,
        # twist_forward,
        motion_manager_node,
        imu_filter_madgwick_node,
        nao_state_publisher_launch,
        # rviz_node,
        # webots_nao_camera_top,
        # webots_nao_camera_bot,
        # send_goal_walk,
    ])
