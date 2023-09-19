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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='False')

    webots_controller_urdf_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'urdf', 'webots_controller.urdf'])
    webots_controller = WebotsController(
        robot_name=os.environ.get('WEBOTS_CONTROLLER_URL'),
        parameters=[{'robot_description': webots_controller_urdf_path}],
        # output='screen',
        remappings=[('participant/CameraTop/image_color', 'image_top'),
                    ('participant/CameraBottom/image_color', 'image_bot'),
                    ('participant/CameraTop/camera_info', 'camera_info_top'),
                    ('participant/CameraBottom/camera_info', 'camera_info_bot')])

    nao_lola_client_node = Node(package='nao_lola_client', executable='nao_lola_client')

    ik_node = Node(package='nao_ik', executable='ik_node')

    nao_phase_provider_node = Node(package='nao_phase_provider', executable='nao_phase_provider',
                                   remappings=[('fsr', 'sensors/fsr')])

    walk_node = Node(package='walk', executable='walk', remappings=[('imu', 'sensors/imu')],
                     parameters=[{'max_forward': 0.4},
                                 {'sole_z': -0.29},
                                 {'sole_x': -0.03}])

    nao_lola_conversion_node = Node(package='nao_lola_conversion', executable='nao_lola_conversion')

    getup_back_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'getupBack.pos'])
    getup_back_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                           name='getup_back', parameters=[{'file': getup_back_pos_path}],
                            remappings=[
                                ('action/_action/feedback', 'getup_back/_action/feedback'),
                                ('action/_action/status', 'getup_back/_action/status'),
                                ('action/_action/cancel_goal', 'getup_back/_action/cancel_goal'),
                                ('action/_action/get_result', 'getup_back/_action/get_result'),
                                ('action/_action/send_goal', 'getup_back/_action/send_goal')
                            ])

    getup_front_pos_path = PathJoinSubstitution(
        [FindPackageShare('wrestle'), 'pos', 'getupFront.pos'])
    getup_front_node = Node(package='naosoccer_pos_action', executable='naosoccer_pos_action',
                            name='getup_front', parameters=[{'file': getup_front_pos_path}],
                            remappings=[
                                ('action/_action/feedback', 'getup_front/_action/feedback'),
                                ('action/_action/status', 'getup_front/_action/status'),
                                ('action/_action/cancel_goal', 'getup_front/_action/cancel_goal'),
                                ('action/_action/get_result', 'getup_front/_action/get_result'),
                                ('action/_action/send_goal', 'getup_front/_action/send_goal')
                            ])

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
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # send_goal_walk = ExecuteProcess(
    #     cmd=['ros2 action send_goal /walk walk_interfaces/action/Walk "{twist: {linear: {x: 0.1}}}"'],
    #     shell=True)

    # robot_detection_node_top = Node(package='wrestle', executable='robot_detection.py',
    #                                 remappings=[('image', 'image_bot'),
    #                                             ('map_point', 'map_point_top')])

    robot_detection_node_bot = Node(package='wrestle', executable='robot_detection.py',
                                    remappings=[('image', 'image_bot'),
                                                ('map_point', 'map_point_bot')])

    base_footprint_node = Node(package='humanoid_base_footprint', executable='base_footprint',
                                remappings=[('walk_support_state', 'phase')])

    # cameratop_tf_publisher = Node(package='tf2_ros', executable='static_transform_publisher',
    #                               arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'CameraTop_optical_frame', 'CameraTop'])

    camerabot_tf_publisher = Node(package='tf2_ros', executable='static_transform_publisher',
                                  arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'CameraBottom_optical_frame', 'CameraBottom'])

    # ipm_node_top_camera = Node(package='ipm_image_node', executable='ipm', name='ipm_top',
    #                            remappings=[('camera_info', 'camera_info_top'),
    #                                        ('input', 'image_top'),
    #                                        ('projected_point_cloud', 'projected_point_cloud_top')],
    #                            parameters=[{'type': 'rgb_image'}])

    # ipm_node_bot_camera = Node(package='ipm_image_node', executable='ipm', name='ipm_bot',
    #                            remappings=[('camera_info', 'camera_info_bot'),
    #                                        ('input', 'image_bot'),
    #                                        ('projected_point_cloud', 'projected_point_cloud_bot')],
    #                            parameters=[{'type': 'rgb_image'}])

    head_skill_node = Node(package='wrestle', executable='head_skill.py')

    # ipm_service_node_top = Node(package='ipm_service', executable='ipm_service', name='ipm_service_top',
    #                             remappings=[('camera_info', 'camera_info_top'),
    #                                         ('map_point', 'map_point_top')])

    ipm_service_node_bot = Node(package='ipm_service', executable='ipm_service', name='ipm_service_bot',
                                remappings=[('camera_info', 'camera_info_bot'),
                                            ('map_point', 'map_point_bot')])

    motion_manager_node_py = Node(package='wrestle', executable='motion_manager.py',
                                #   output='screen'
                                remappings=[('imu', 'sensors/filtered_imu')],
                                  )

    arm_provider_node = Node(package='wrestle', executable='arm_provider.py')

    crouch_node = Node(package='wrestle', executable='crouch.py',
                    #    output='screen'
                       )

    return LaunchDescription([
        rviz_launch_arg,
        motion_manager_node_py,
        webots_controller,
        nao_lola_client_node,
        ik_node,
        nao_phase_provider_node,
        walk_node,
        nao_lola_conversion_node,
        getup_back_node,
        getup_front_node,
        imu_filter_madgwick_node,  # Slow
        nao_state_publisher_launch,  # Slow
        rviz_node,
        # send_goal_walk,
        # robot_detection_node_top,
        robot_detection_node_bot,
        base_footprint_node,
        # cameratop_tf_publisher,
        camerabot_tf_publisher,  # Feels slow
        # ipm_node_top_camera,
        # ipm_node_bot_camera,
        head_skill_node,
        # ipm_service_node_top,
        ipm_service_node_bot,
        arm_provider_node,
        crouch_node,
    ])
