from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='cartesian_impedance_example_controller',
        description='Controller to use'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz'
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run in headless mode'
    )

    # Include franka_gazebo launch
    franka_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_gazebo'),
                'launch',
                'panda.launch.py'
            ])
        ]),
        launch_arguments={
            'controller': LaunchConfiguration('controller'),
            'rviz': LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # Robot interface node
    robot_interface = Node(
        package='fep_rl_experiment_ros2',
        executable='robot_interface',
        name='robot_interface',
        output='screen'
    )

    # Dummy image publisher
    image_publisher = Node(
        package='fep_rl_experiment_ros2',
        executable='dummy_image_publisher',
        name='image_publisher',
        output='screen'
    )

    # Dummy cube publisher
    cube_publisher = Node(
        package='fep_rl_experiment_ros2',
        executable='dummy_cube_publisher',
        name='cube_publisher',
        output='screen',
        remappings=[
            ('pose', '/cube_pose')
        ]
    )

    # Static transform publisher (camera_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            '1.176', '0.001', '0.483',
            '0.65765434', '0.65382486', '-0.2637626', '-0.26539238',
            'panda_link0', 'camera_link'
        ]
    )

    return LaunchDescription([
        controller_arg,
        rviz_arg,
        headless_arg,
        franka_gazebo,
        robot_interface,
        image_publisher,
        cube_publisher,
        static_tf
    ])
