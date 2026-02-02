from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.1.11',
        description='IP address of the Franka robot'
    )

    marker_ids_arg = DeclareLaunchArgument(
        'markerIds',
        default_value='571 581 591 601 611 621',
        description='ArUco marker IDs'
    )

    marker_size_arg = DeclareLaunchArgument(
        'markerSize',
        default_value='0.042',
        description='ArUco marker size in meters'
    )

    cube_size_arg = DeclareLaunchArgument(
        'cubeSize',
        default_value='0.05',
        description='Cube size in meters'
    )

    session_id_arg = DeclareLaunchArgument(
        'sessionId',
        default_value='training_session',
        description='Session ID for logging'
    )

    trajectory_length_arg = DeclareLaunchArgument(
        'trajectoryLength',
        default_value='250',
        description='Trajectory length'
    )

    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.05',
        description='Control timestep'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera frame ID'
    )

    ref_frame_arg = DeclareLaunchArgument(
        'ref_frame',
        default_value='camera_link',
        description='Reference frame for ArUco'
    )

    # Include Franka controller launch
    franka_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_example_controllers'),
                'launch',
                'cartesian_impedance_example_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip')
        }.items()
    )

    # Include RealSense camera launch
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'enable_depth': 'true',
            'enable_pointcloud': 'true',
            'color_fps': '30.0'
        }.items()
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('markerSize'),
            'cube_size': LaunchConfiguration('cubeSize'),
            'marker_ids': LaunchConfiguration('markerIds'),
            'reference_frame': LaunchConfiguration('ref_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'marker_frame': 'aruco_marker_frame',
            'corner_refinement': 'LINES'
        }],
        remappings=[
            ('/camera_info', '/camera/color/camera_info'),
            ('/image', '/camera/color/image_raw')
        ]
    )

    # Online learning node
    online_learning = Node(
        package='fep_rl_experiment_ros2',
        executable='online_learning',
        name='online_learning',
        output='screen',
        parameters=[{
            'session_id': LaunchConfiguration('sessionId'),
            'trajectory_length': LaunchConfiguration('trajectoryLength'),
            'dt': LaunchConfiguration('dt')
        }]
    )

    # Static transform publisher (camera_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            '0.915', '-0.065', '0.32',
            '-0.2503586', '-0.0032775', '0.968064', '-0.0126727',
            'panda_link0', 'camera_link'
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        marker_ids_arg,
        marker_size_arg,
        cube_size_arg,
        session_id_arg,
        trajectory_length_arg,
        dt_arg,
        camera_frame_arg,
        ref_frame_arg,
        franka_controller,
        realsense,
        aruco_node,
        online_learning,
        static_tf
    ])
