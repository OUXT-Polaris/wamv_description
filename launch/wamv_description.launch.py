# Copyright (c) 2020 OUXT Polaris
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

import launch_ros.actions
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

share_dir_path = os.path.join(get_package_share_directory('wamv_description'))
urdf_path = os.path.join(share_dir_path, 'urdf', 'wamv.urdf')


def generate_robot_description(enable_dummy: bool):
    if enable_dummy:
        xacro_path = os.path.join(
            share_dir_path, 'urdf', 'wamv_dummy.urdf.xacro')
    else:
        xacro_path = os.path.join(
            share_dir_path, 'urdf', 'wamv.urdf.xacro')
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='  ')
    return robot_desc


def generate_launch_description():
    enable_dummy = LaunchConfiguration('enable_dummy', default=True)
    enable_dummy_arg = DeclareLaunchArgument(
        'enable_dummy', default_value=enable_dummy, description="if true, enable dummy wam-v.")
    controller_config = os.path.join(get_package_share_directory(
        'wamv_description'), 'config', 'controllers.yaml')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  arguments=[urdf_path],
                                  condition=UnlessCondition(enable_dummy),
                                  parameters=[{'robot_description': generate_robot_description(False)}])

    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': generate_robot_description(False)}, controller_config],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        condition=UnlessCondition(enable_dummy))

    rsp_dummy = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        condition=IfCondition(enable_dummy),
        arguments=[urdf_path],
        parameters=[{'robot_description': generate_robot_description(True)}])

    control_node_dummy = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': generate_robot_description(True)}, controller_config],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        condition=IfCondition(enable_dummy)
    )

    return launch.LaunchDescription([rsp,
                                     rsp_dummy,
                                     enable_dummy_arg,
                                     control_node,
                                     control_node_dummy,
                                     ExecuteProcess(
                                         cmd=[
                                             'ros2',
                                             'control',
                                             'load_controller',
                                             'joint_state_broadcaster'],
                                         output='screen',
                                         shell=True,
                                     ),
                                     ExecuteProcess(
                                         cmd=[
                                             'ros2',
                                             'control',
                                             'load_controller',
                                             'usv_joy_controller'],
                                         output='screen',
                                         shell=True,
                                     ),
                                     ])
