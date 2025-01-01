from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Retrieving path information
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# Working with env variables
from launch.actions import SetEnvironmentVariable

# Simulating event handling
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# Path Variables
ignition_ros_package_path = get_package_share_directory("ros_ign_gazebo")
udemy_ros2_pkg_path = get_package_share_directory("udemy_ros2_pkg")
simulation_world_file_path = Path(udemy_ros2_pkg_path, "worlds", "project_world.sdf")
simulation_models_path = Path(udemy_ros2_pkg_path, "models")

print(f"simulation_models_path: {simulation_models_path}")

simulation = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', '--render-engine', 'ogre', simulation_world_file_path],
    output='screen'
)

# Start rqt
rqt_process = ExecuteProcess(
    cmd=['rqt'],
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=simulation_models_path
        ),
        simulation,
        rqt_process,
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/camera_rod_pos_cmd@std_msgs/msg/Float64@ignition.msgs.Double"
            ],
            remappings=[("camera","/camera/image_raw")],
            output="screen"
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=simulation,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=simulation,
                on_exit=[
                    rqt_process
                ]
            )
        )
    ])


