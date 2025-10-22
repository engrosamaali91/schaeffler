# isaac_and_nav2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    usd_path = LaunchConfiguration("usd_path")
    play_sim_on_start = LaunchConfiguration("play_sim_on_start")
    map_yaml = LaunchConfiguration("map")

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
        default_value="src/nav_bringup/maps/slam_map.yaml",
        description="Map YAML for Nav2",
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
        }.items(),
    )

    # Delay Nav2 slightly so Isaac Sim has time to bring up its graph/topics
    delayed_nav2 = TimerAction(period=15.0, actions=[nav2_launch])

    return LaunchDescription([
        declare_usd, declare_play, declare_map,
        isaac_launch,
        delayed_nav2,
    ])
