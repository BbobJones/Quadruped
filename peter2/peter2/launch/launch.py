import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

# Function to generate the launch description
def generate_launch_description():

    # Check if we're told to use simulation time; use_sim_time is a launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Specify the full path to the URDF file
    sdf_file_name = 'peter1.sdf'
    rviz_config = 'peter1.rviz'
    path_to_sdf = os.path.join(
        get_package_share_directory('peter2'),
        'sdf',
        sdf_file_name)
    
    path_to_world = os.path.join(
        get_package_share_directory('peter2'),
        'sdf',
        'peter_world.sdf')
        
    path_to_rviz = os.path.join(
        get_package_share_directory('peter2'),
        'rviz',
        rviz_config)
    
    world_launch = f"-r -v 4 {path_to_world}"

    # Set the environment variable for GZ_SIM_RESOURCE_PATH, which tells Gazebo where to find resources
    resource_path = get_package_share_directory('peter2')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # Create a robot_state_publisher node to publish the robot's state, especially joint states and TF information
    node_robot_state_publisher = Node(
        package='robot_state_publisher',  # Use the robot_state_publisher package
        executable='robot_state_publisher',  # The executable name
        name='robot_state_publisher',  # The node name
        output='screen',  # Output will be printed to the screen
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_sdf)]), value_type=str)
        }]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),  # Get the path to the ros_gz_sim package
                "launch",  # Launch folder
                "gz_sim.launch.py"  # The Gazebo launch file
            )
        ),
        launch_arguments={"gz_args": world_launch}.items(),  # Arguments to pass to Gazebo: load empty.sdf with verbose logging
    )

    # Spawn the robot in Gazebo using the create service equivalent in ros_gz_sim
    spawn_entity = Node(
        package="ros_gz_sim",  # Use the ros_gz_sim package
        executable="create",  # The 'create' executable to spawn models in Gazebo
        arguments=[
            "-name", "peter1",  # The name of the robot in the Gazebo simulation
            "-file", path_to_sdf,  # Path to the URDF file that defines the robot
            "-x", "0", "-y", "0", "-z", "0.04"  # Initial position of the robot in the Gazebo world (x, y, z)
        ],
        output="screen",  # Print the output to the screen
    )

    # Add the ros_gz_bridge node to bridge joint states and commands between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',  # Use the ros_gz_bridge package
        executable='parameter_bridge',  # Run the parameter bridge executable
        name='ros_gz_bridge',  # Name of the bridge node
        arguments=[
            # Bridge the /joint_states topic from ROS 2 (sensor_msgs/JointState) to Gazebo (gz.msgs.Model)
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



            # Bridge velocity commands (if controlling with velocity) between ROS 2 and Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/lidar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/map@map_msgs/msg/map[gz.msgs.map',

        ],
        output='screen'  # Print the output to the screen
    )
    
        # run Joint controller in a new terminal window
    joint_controller_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c', 
            'ros2 run peter2 joint_controller; exec bash'
        ],
        output='screen'
    )
        # run image_listener in a new terminal window
    image_listener_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c', 
            'ros2 run peter2 image_listener; exec bash'
        ],
        output='screen'
    )
    

    gmapping_node = Node(
     package='slam_toolbox',
     executable='async_slam_toolbox_node',
     name='slam_toolbox',
     output='screen',
     parameters=[
        {'use_sim_time': True},
        {'scan_topic': '/lidar_scan'}  
    ]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', path_to_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )



    # Return the full launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',  # Declare the use_sim_time argument
            default_value='false',  # Default to false (no simulation time)
            description='Use simulation time if true'  # Description of the argument
        ),

        # Set the environment variable for GZ_SIM_RESOURCE_PATH
        set_gz_sim_resource_path,

        # Start the joint state publisher, robot state publisher, Gazebo simulation, and spawn the robot
        node_robot_state_publisher,
        gz_sim,
        gmapping_node,
        spawn_entity,
        bridge,  # Add the ros_gz_bridge node
        rviz_node,
        joint_controller_node,  # Add joint_controller node
        image_listener_node,# Add image_listener(openCV) node
    ])


