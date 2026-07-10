from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "sabry",
        package_name="sabry_moveit"
    ).to_moveit_configs()

    demo = generate_demo_launch(moveit_config)

    drill_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drill_controller"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(demo)
    ld.add_action(drill_spawner)

    return ld