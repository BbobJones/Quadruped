import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths
    sdf_file_name = 'peter1.sdf'
    rviz_config = 'peter1.rviz'
    path_to_sdf = os.path.join(get_package_share_directory('peter2'), 'sdf', sdf_file_name)
    path_to_world = os.path.join(get_package_share_directory('peter2'), 'sdf', 'peter_world.sdf')
    path_to_rviz = os.path.join(get_package_share_directory('peter2'), 'rviz', rviz_config)
    resource_path = get_package_share_directory('peter2')
    world_launch = f"-r -v 4 {path_to_world}"

    # Environment variable for resource path
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(path_to_sdf)]),
                    value_type=str
                )
            }
        ]
    )

    # Gazebo launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": world_launch}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "peter1", "-file", path_to_sdf, "-x", "0", "-y", "0", "-z", "0.04"],
        output="screen"
    )

    # ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/boxes_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/jlf1_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jlf2_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jlf3_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrf1_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrf2_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrf3_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jlr1_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jlr2_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jlr3_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrr1_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrr2_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/jrr3_topic@std_msgs/msg/Float64@gz.msgs.Double',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar_scan1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/map@map_msgs/msg/map[gz.msgs.map',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'

        ],
        output='screen'
    )

    # Joint controller
    joint_controller_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run peter2 joint_controller; exec bash'],
        output='screen'
    )

    # Image listener
    image_listener_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run peter2 image_listener; exec bash'],
        output='screen'
    )

    # SLAM Toolbox

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', path_to_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # EKF node
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('peter2'), 'params', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/odom', '/odom1')],
    )

    # Return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        set_gz_sim_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        joint_controller_node,
        image_listener_node,
        rviz_node,
        ekf,
    ])



