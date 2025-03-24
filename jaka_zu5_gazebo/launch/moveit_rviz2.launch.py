from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
  
    # MoveIt 配置
    moveit_config = MoveItConfigsBuilder("jaka_zu5", package_name="jaka_zu5_moveit_config").to_moveit_configs()
    ld = LaunchDescription()
    # 启动 MoveIt 中的 move_group 节点
    my_generate_move_group_launch(ld, moveit_config)
 
    # 启动 RViz 并加载 MoveIt 配置
    my_generate_moveit_rviz_launch(ld, moveit_config)
 
    return ld
 
def my_generate_move_group_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
 
    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
 
    move_group_configuration = {
        "robot_description": moveit_config.robot_description,
        "robot_description_semantic": moveit_config.robot_description_semantic,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),  # 使用 capabilities 参数
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
  
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
  }
    move_group_params = [moveit_config.to_dict(), move_group_configuration]
    move_group_params.append({"use_sim_time": True})
 
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=move_group_params,
        output="screen",
        extra_debug_args=["--debug"] if LaunchConfiguration("debug") == "true" else []
    )
 
def my_generate_moveit_rviz_launch(ld, moveit_config):
   
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
   
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description,
        {"use_sim_time": True}
    ]
    
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        arguments=['-d', LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        output="log"
    )