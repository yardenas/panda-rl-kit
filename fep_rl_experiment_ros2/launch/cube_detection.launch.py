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

    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Load gripper'
    )

    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='panda',
        description='Arm ID'
    )

    transmission_arg = DeclareLaunchArgument(
        'transmission',
        default_value='position',
        description='Transmission type'
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

    # Include franka_control launch
    franka_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_control'),
                'launch',
                'franka_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'arm_id': LaunchConfiguration('arm_id')
        }.items()
    )

    # Include ROS controllers
    ros_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_moveit_config'),
                'launch',
                'ros_controllers.launch.py'
            ])
        ]),
        launch_arguments={
            'transmission': LaunchConfiguration('transmission')
        }.items()
    )

    # Include MoveIt demo
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'load_robot_description': 'false',
            'pipeline': 'ompl',
            'moveit_controller_manager': 'simple'
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
            'align_depth': 'true',
            'depth_width': '640',
            'depth_height': '480',
            'color_width': '640',
            'color_height': '480',
            'depth_fps': '15.0',
            'color_fps': '15.0'
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

    # Static transform publisher (camera_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link',
        arguments=[
            '0.1052007131337004', '-0.362021457569345', '0.4078846634858508',
            '-0.06978692995692472', '0.29127019481885674', '0.2774807147818239', '0.9128504318570158',
            'panda_link0', 'camera_link'
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        load_gripper_arg,
        arm_id_arg,
        transmission_arg,
        marker_ids_arg,
        marker_size_arg,
        cube_size_arg,
        camera_frame_arg,
        ref_frame_arg,
        franka_control,
        ros_controllers,
        moveit_demo,
        realsense,
        aruco_node,
        static_tf
    ])
