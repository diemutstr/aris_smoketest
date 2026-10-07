"""One arm: the robot model, ros2_control with the Franka hardware, and its controllers.

    ros2 launch aris_bringup arm.launch.py args:=<robot/generated/arm_31.json>

The argument file is written by `aris-robot serve` (bringup.py) from rig.json and the site; nothing
about the arm is typed here.  Everything runs in the namespace arm_<id> and on the DDS domain
of the arm.  Started:
  robot_state_publisher, ros2_control_node (franka_hardware, 1 kHz), joint_state_publisher,
  joint_state_broadcaster, franka_robot_state_broadcaster (not with fake hardware),
  fr3_arm_controller (active): the one controller that moves the arm.
Not started, on purpose: MoveIt, RViz, and the gripper node (homing the gripper would open
the fingers that hold the pen holder).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _nodes(context):
    path = LaunchConfiguration('args').perform(context)
    with open(path) as f:
        a = json.load(f)
    if not a['mounted'] and not a['use_fake_hardware']:
        raise RuntimeError(f"arm {a['arm']} is not mounted in site.json; refusing to connect "
                           f"to {a['robot_ip']}")
    ns = a['namespace']
    fr3 = os.path.join(get_package_share_directory('franka_description'),
                       'robots', 'fr3', 'fr3.urdf.xacro')
    description = xacro.process_file(fr3, mappings={
        'ros2_control': 'true',
        'robot_ip': a['robot_ip'],
        'hand': 'true',
        'ee_id': 'franka_hand',
        'use_fake_hardware': str(a['use_fake_hardware']).lower(),
        'fake_sensor_commands': 'false',
        # the operator patches (Aris_Kindt/operator_franka_patches/fr3.urdf.xacro)
        'mount_to_world': str(a['mount_to_world']).lower(),
        'mroll': repr(a['mroll']), 'mpitch': repr(a['mpitch']),
        'myaw': repr(a['myaw']), 'mz': repr(a['mz']),
    }).toprettyxml(indent='  ')
    robot_description = {'robot_description': description}
    cm = f'/{ns}/controller_manager'

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner', namespace=ns,
                    arguments=[name, '--controller-manager', cm,
                               '--controller-manager-timeout', '60', *extra],
                    output='screen')

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=ns, parameters=[robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', namespace=ns,
             parameters=[robot_description, a['controllers']],
             remappings=[('joint_states', 'franka/joint_states')],
             output='screen', on_exit=Shutdown()),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace=ns, parameters=[{'source_list': ['franka/joint_states'], 'rate': 30}]),
        spawner('joint_state_broadcaster'),
        spawner('fr3_arm_controller'),
    ]
    if not a['use_fake_hardware']:
        nodes.append(spawner('franka_robot_state_broadcaster'))
    env = [SetEnvironmentVariable('ROS_DOMAIN_ID', str(a['domain'])),
           SetEnvironmentVariable('RMW_IMPLEMENTATION', a['rmw'])]
    return env + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('args', description='arm_<slot>.json, written by `aris-robot serve`'),
        OpaqueFunction(function=_nodes),
    ])
