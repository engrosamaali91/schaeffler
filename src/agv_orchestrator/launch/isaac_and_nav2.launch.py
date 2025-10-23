# isaac_and_nav2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    usd_path = LaunchConfiguration("usd_path")
    play_sim_on_start = LaunchConfiguration("play_sim_on_start")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")                 # NEW
    rviz_config = LaunchConfiguration("rviz_config")   # NEW

    # package shares
    nav_bringup_share = FindPackageShare("nav_bringup")
    orchestrator_share = FindPackageShare("agv_orchestrator") 


    declare_usd = DeclareLaunchArgument(
        "usd_path",
        default_value="/home/schaeffler/Downloads/omron_emma/emma.usd",
        description="Path to the Omron Emma USD file for Isaac Sim",
    )
    declare_play = DeclareLaunchArgument(
        "play_sim_on_start",
        default_value="true",
        description="Start the scene in play mode",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([nav_bringup_share, "maps", "slam_map.yaml"]),
        description="Map YAML for Nav2 (defaults to nav_bringup package map)",
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([nav_bringup_share, "config", "nav2_params.yaml"]),
        description="Nav2 parameters YAML file",
    )

    # NEW: rviz toggle + config path
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2 (true/false)",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([orchestrator_share, "rviz", "config.rviz"]),
        description="RViz2 config file",
    )

    # Isaac Sim launcher (from IsaacSim ROS workspace)
    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("isaacsim"), "launch", "run_isaacsim.launch.py"])
        ),
        launch_arguments={
            "gui": usd_path,                 # Isaac launch expects 'gui' arg = path to USD
            "play_sim_on_start": play_sim_on_start,
        }.items(),
    )

    # Your Nav2 bringup (from your own package)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_bringup"), "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    # Delay Nav2 slightly so Isaac Sim has time to bring up its graph/topics
    delayed_nav2 = TimerAction(period=20.0, actions=[nav2_launch])
    # NEW: RViz2 node (guarded by rviz toggle)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_usd, declare_play, declare_map, declare_params,
        declare_rviz, declare_rviz_config,
        isaac_launch,
        delayed_nav2,
        rviz_node,
    ])
