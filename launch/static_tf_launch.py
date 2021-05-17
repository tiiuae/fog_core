from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    UAV_NAME = os.getenv('UAV_NAME')

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

    return ld
