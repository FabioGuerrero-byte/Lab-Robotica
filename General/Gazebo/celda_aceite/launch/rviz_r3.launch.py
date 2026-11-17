import os
from math import cos, radians, sin

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('celda_aceite')
    r = yaml.safe_load(open(os.path.join(share, 'config', 'robots.yaml')))['robots']['robot3']
    urdf = os.path.join(get_package_share_directory(r['pkg']), r['urdf'])
    xml = xacro.process_file(urdf).toxml() if urdf.endswith('.xacro') else open(urdf).read()
    yaw = radians(r.get('yaw_deg', 0.0)); bx, by = r.get('base_cm', [0.0, 0.0])
    x = (r['x_cm'] - (cos(yaw) * bx - sin(yaw) * by)) / 100.0
    y = (r['y_cm'] - (sin(yaw) * bx + cos(yaw) * by)) / 100.0
    z = r.get('z_cm', 0.0) / 100.0
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='robot3_real',
             parameters=[{'robot_description': ParameterValue(xml, value_type=str),
                          'frame_prefix': 'robot3_real/', 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'world', '--child-frame-id', 'robot3_real/world']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config', 'r3.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
