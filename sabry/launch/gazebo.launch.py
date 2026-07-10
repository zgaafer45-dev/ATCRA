import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    sabry = get_package_share_directory("sabry")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        sabry, "urdf", "sabry.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(sabry).parent.resolve())
            ]
        )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={"gz_args": "-v 4 -r empty.sdf"}.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "sabry"],
    )

    

# Spawn entity **after 3 seconds** delay to ensure Gazebo is up
#     gz_spawn_entity = TimerAction(
#     period=3.0,
#     actions=[Node(
#         package="ros_gz_sim",
#         executable="create",
#         name='spawn_entity',
#         output="screen",
#         arguments=["-topic", "robot_description", "-name", "sabry"],
#         remappings=[("/world/empty/create", "/spawn_entity")]
#     )]
# )

    gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        # Clock topic (Gazebo → ROS)
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

        # SpawnEntity service (ROS → Gazebo)
        "/world/empty/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean",

        # DeleteEntity service (ROS → Gazebo)
        "/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity@gz.msgs.Entity@gz.msgs.Boolean"
    ],
    output="screen"
)

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])