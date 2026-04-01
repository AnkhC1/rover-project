import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import EmitEvent, RegisterEventHandler, IncludeLaunchDescription
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import lifecycle_msgs.msg
import launch

def generate_launch_description():
    pkg_path = get_package_share_directory('rover_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_file = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    slam_config = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. Static TF: base_link → laser_frame
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.1',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser_frame']
    )

    # 2. EKF: fuses /odom → publishes odom → base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[{
            'frequency': 30.0,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom',
            'odom0_config': [True,  True,  False,
                             False, False, True,
                             False, False, False,
                             False, False, True,
                             False, False, False],
            'odom0_differential': False,
            'odom0_relative': False,
        }]
    )

    # 3. SLAM toolbox: builds the map and localizes simultaneously
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_config],
        output='screen'
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    slam_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    slam_configured_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[slam_activate]
        )
    )

    # 4. micro-ROS agent: bridges ESP32
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888']
    )

    # 5. Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        tf_base_to_laser,
        ekf_node,
        slam_node,
        slam_configure,
        slam_configured_handler,
        micro_ros_agent_node,
        nav2_bringup,
    ])
