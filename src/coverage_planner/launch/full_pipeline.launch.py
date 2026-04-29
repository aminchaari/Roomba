"""
full_pipeline.launch.py
───────────────────────
Single entry-point launch file for the complete Roomba pipeline:

  Phase 1 — Frontier Exploration
    • Gazebo simulation
    • SLAM Toolbox
    • Nav2 stack
    • Frontier Explorer node

  Phase 2 — Automatic handoff (triggered by PipelineManager)
    • PipelineManager  → watches logs, saves map, signals CPP
    • CoveragePlanner  → waits for signal + AMCL, then sweeps the map

Usage:
    ros2 launch coverage_planner full_pipeline.launch.py
    ros2 launch coverage_planner full_pipeline.launch.py headless:=true rviz:=false

Notes:
  • Adjust the 'explorer_pkg' and 'explorer_launch' variables below if the
    frontier-explorer package has a different name in your workspace.
  • AMCL is launched here in "map" mode (loads the saved map).  If you
    prefer to re-use the SLAM localisation TF tree instead, set
    use_amcl:=false and the CoveragePlanner will still work (it will warn
    once if /amcl_pose never arrives, then proceed anyway).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Tuneable constants ────────────────────────────────────────────────────── #

# Package name of the frontier explorer (the ros2-autonomous-explorer repo)
EXPLORER_PKG    = 'autonomous_explorer'
EXPLORER_LAUNCH = 'auto_explore.launch.py'

# Coverage-planner package (your package)
CPP_PKG = 'coverage_planner'

# Map save location (must match pipeline_manager.py defaults)
MAP_DIR  = os.path.expanduser('~/roomba_ws/maps')
MAP_NAME = 'explored_map'
MAP_YAML = os.path.join(MAP_DIR, MAP_NAME + '.yaml')

# Delay (seconds) before starting PipelineManager + CoveragePlanner
# (they are passive listeners, so starting early is fine)
MANAGER_START_DELAY = 5.0


# ──────────────────────────────────────────────────────────────────────────── #

def generate_launch_description():

    # ── Declare arguments ─────────────────────────────────────────────────── #
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo in headless (server-only) mode'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2'
    )
    use_amcl_arg = DeclareLaunchArgument(
        'use_amcl', default_value='true',
        description='Launch AMCL for localisation after map is saved'
    )
    step_size_arg = DeclareLaunchArgument(
        'step_size', default_value='4',
        description='Coverage sweep row spacing in grid cells (4 cells × 0.05m/cell = 0.20m spacing)'
    )
    coverage_algorithm_arg = DeclareLaunchArgument(
        'coverage_algorithm', default_value='grid',
        description='Coverage path planner to use: grid or fields2cover (experimental, slow on large maps)'
    )
    robot_width_arg = DeclareLaunchArgument(
        'robot_width', default_value='0.34',
        description='Robot width for Fields2Cover planning (meters)'
    )
    robot_length_arg = DeclareLaunchArgument(
        'robot_length', default_value='0.40',
        description='Robot length for Fields2Cover planning (meters)'
    )
    robot_min_radius_arg = DeclareLaunchArgument(
        'robot_min_radius', default_value='0.30',
        description='Minimum turning radius for Fields2Cover planning (meters)'
    )
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area', default_value='0.25',
        description='Minimum contour area to keep when extracting free-space boundaries (m^2)'
    )
    map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir', default_value=MAP_DIR,
        description='Directory where the map will be saved'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value=MAP_NAME,
        description='Base name (no extension) for the saved map'
    )

    headless   = LaunchConfiguration('headless')
    rviz       = LaunchConfiguration('rviz')
    use_amcl   = LaunchConfiguration('use_amcl')
    step_size  = LaunchConfiguration('step_size')
    coverage_algorithm = LaunchConfiguration('coverage_algorithm')
    robot_width = LaunchConfiguration('robot_width')
    robot_length = LaunchConfiguration('robot_length')
    robot_min_radius = LaunchConfiguration('robot_min_radius')
    min_contour_area = LaunchConfiguration('min_contour_area')
    map_save_dir = LaunchConfiguration('map_save_dir')
    map_name     = LaunchConfiguration('map_name')

    # ── Phase 1: Frontier exploration stack ───────────────────────────────── #
    frontier_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(EXPLORER_PKG),
            '/launch/',
            EXPLORER_LAUNCH,
        ]),
        launch_arguments={
            'headless': headless,
            'rviz':     rviz,
        }.items(),
    )

    # ── Phase 2: Pipeline manager (starts early, idles until log message) ─── #
    pipeline_manager_node = TimerAction(
        period=MANAGER_START_DELAY,
        actions=[
            LogInfo(msg='[Pipeline] Starting PipelineManager node...'),
            Node(
                package=CPP_PKG,
                executable='pipeline_manager',
                name='pipeline_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_save_dir': map_save_dir,
                    'map_name':     map_name,
                }],
            ),
        ]
    )

    # ── Phase 2: Coverage Planner (idles until /exploration_done arrives) ─── #
    coverage_planner_node = TimerAction(
        period=MANAGER_START_DELAY,
        actions=[
            LogInfo(msg='[Pipeline] Starting CoveragePlanner node (idle until handoff)...'),
            Node(
                package=CPP_PKG,
                executable='coverage_planner',
                name='coverage_planner',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'step_size':    step_size,
                    'coverage_algorithm': coverage_algorithm,
                    'robot_width': robot_width,
                    'robot_length': robot_length,
                    'robot_min_radius': robot_min_radius,
                    'min_contour_area': min_contour_area,
                    'amcl_timeout': 30.0,
                }],
            ),
        ]
    )

    # ── Assemble LaunchDescription ────────────────────────────────────────── #
    return LaunchDescription([
        # Arguments
        headless_arg,
        rviz_arg,
        use_amcl_arg,
        step_size_arg,
        coverage_algorithm_arg,
        robot_width_arg,
        robot_length_arg,
        robot_min_radius_arg,
        min_contour_area_arg,
        map_save_dir_arg,
        map_name_arg,

        # Phase 1 — exploration
        LogInfo(msg='[Pipeline] Phase 1: Launching frontier exploration stack...'),
        frontier_exploration,

        # Phase 2 — manager + CPP (passive, start early)
        pipeline_manager_node,
        coverage_planner_node,
    ])