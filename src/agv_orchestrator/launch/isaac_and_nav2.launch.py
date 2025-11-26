# isaac_and_nav2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    usd_path = LaunchConfiguration("usd_path")
    play_sim_on_start = LaunchConfiguration("play_sim_on_start")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")                 
    rviz_config = LaunchConfiguration("rviz_config")   
    run_test = LaunchConfiguration("run_test")
    compute_kpi = LaunchConfiguration("compute_kpi")
    compute_kpi_script = LaunchConfiguration("compute_kpi_script")

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

    # rviz toggle and config path
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2 (true/false)",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([orchestrator_share, "rviz", "config.rviz"]),
        description="RViz2 config file",
    )
    declare_run_test = DeclareLaunchArgument(
    "run_test",
    default_value="false",
    description="Run nav2_test.py after Nav2 is active (true/false)",
    )
    declare_compute_kpi = DeclareLaunchArgument(
    "compute_kpi",
    default_value="false",
    description="Run compute_kpi_normalized.py after nav2_test finishes (true/false)",
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

    # run Python script (installed via CMake) as a node
    nav2_test_node = Node(
        package="agv_orchestrator",
        executable="nav2_test.py",
        name="nav2_test",
        output="screen",
        condition=IfCondition(run_test),
    )

    # Delay your script until after Nav2 delay (20s) + a little buffer (10s)
    delayed_nav2_test = TimerAction(period=30.0, actions=[nav2_test_node])


    # KPI script path (installed in share/agv_orchestrator/scripts)
    declare_compute_kpi_script = DeclareLaunchArgument(
        "compute_kpi_script",
        default_value=PathJoinSubstitution(
            [orchestrator_share, "scripts", "compute_kpi_normalized.py"]
        ),
        description="Path to compute_kpi_normalized.py to execute",
    )


    # Execute KPI script using system python3 when nav2_test exits
    compute_kpi_exec = ExecuteProcess(
        cmd=["python3", compute_kpi_script],
        output="screen",
        condition=IfCondition(compute_kpi),
    )
    # After KPI script finishes, shut down the whole launch
    shutdown_on_kpi_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=compute_kpi_exec,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    # Run KPI when nav2_test process exits
    run_kpi_on_test_exit = RegisterEventHandler(
        OnProcessExit(target_action=nav2_test_node, on_exit=[compute_kpi_exec])
    )

    return LaunchDescription([
        declare_usd, declare_play, declare_map, declare_params,
        declare_rviz, declare_rviz_config,
        declare_run_test,
        isaac_launch,
        delayed_nav2,
        rviz_node,
        delayed_nav2_test,
        declare_compute_kpi,
        declare_compute_kpi_script,
        run_kpi_on_test_exit,
        shutdown_on_kpi_exit,
    ])
