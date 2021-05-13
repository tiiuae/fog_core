import launch
from launch_ros.actions import Node
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    # environment variables
    UAV_NAME = os.getenv('UAV_NAME')

    # arguments
    ld.add_action(launch.actions.DeclareLaunchArgument("rplidar_mode", default_value="outdoor"))

    # mode select for rplidar
    # ----------------------
    # Sensitivity: optimized for longer ranger, better sensitivity but weak environment elimination 
    # Boost: optimized for sample rate 
    # Stability: for light elimination performance, but shorter range and lower sample rate 
    rplidar_mode = launch.substitutions.PythonExpression(['"Stability" if "outdoor" == "', launch.substitutions.LaunchConfiguration("rplidar_mode"), '" else "Sensitivity"'])

    #namespace declarations
    namespace = UAV_NAME

    # frame names
    fcu_frame = UAV_NAME + "/fcu"
    rplidar_frame = UAV_NAME + "/rplidar"
    garmin_frame = UAV_NAME + "/garmin"
    
    # node definitions
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_rplidar_static_transform_publisher",
            arguments = ["0", "0", "0.09", "0", "0", "0", fcu_frame, rplidar_frame],
            output='screen',
        ),
    )
    
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_garmin_static_transform_publisher",
            arguments = ["-0.007", "-0.05", "-0.036", "0", "0", "0", fcu_frame, garmin_frame],
            output='screen',
        ),
    )
    
    ld.add_action(
        Node(
            name = 'rplidar',
            package = 'rplidar_ros2',
            executable = 'rplidar',
            parameters = [{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,  # A3
                'frame_id': rplidar_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': rplidar_mode,
            }],
            output = 'screen',
        ),
    )

    return ld
