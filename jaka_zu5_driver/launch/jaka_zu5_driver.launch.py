import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    jaka_zu5_config = os.path.join(
        get_package_share_directory('jaka_zu5_driver'),
        'config',
        'controller.yaml'
    )
    jaka_zu5_driver_node = Node(
                                 package="jaka_zu5_driver",
                                 executable="jaka_zu5_driver_node",
                                 parameters=[
                                {
                                    "ip": "169.254.232.101",
                                    "controller_yaml_path": jaka_zu5_config,
                                    "algo_log_path": "/root/RCC_WS/log/ROBOT_ARM",
                                    "restart_admit_after_adjust": False,
                                    "admit_adjust_after_inpos": True,
                                    "admit_depth_thres": 0.5,
                                    "admit_force_thres": -3.0,
                                    "model":"jaka_arm",
                                }],
                                remappings=[("/jaka_zu5_driver/joint_position","/joint_states")]
    )
    

    return launch.LaunchDescription(
        [
            jaka_zu5_driver_node
        ]
        )
