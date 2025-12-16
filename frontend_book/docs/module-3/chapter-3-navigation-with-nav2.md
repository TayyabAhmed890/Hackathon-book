---
sidebar_position: 4
---

# Navigation with Nav2

## Path Planning Fundamentals

Navigation for humanoid robots requires specialized approaches due to their unique locomotion characteristics. Unlike wheeled robots, humanoid robots must consider balance, step constraints, and bipedal gait patterns when planning paths.

### Navigation Stack Overview

The Navigation2 (Nav2) stack provides a comprehensive framework for robot navigation:

- **Global Planner**: Creates optimal paths from start to goal
- **Local Planner**: Handles real-time obstacle avoidance and path following
- **Controller**: Translates navigation commands to robot motion
- **Recovery Behaviors**: Handles situations where the robot gets stuck
- **Costmap Management**: Maintains representations of the environment

### Key Navigation Concepts

- **Costmaps**: Grid-based representations of the environment with cost values
- **Trajectory Generation**: Creating smooth, executable paths for the robot
- **Dynamic Obstacle Handling**: Responding to moving obstacles in real-time
- **Localization Integration**: Using position estimates for navigation

## Navigation for Bipedal Humanoids

Humanoid navigation presents unique challenges that require specialized approaches:

### Bipedal-Specific Constraints

- **Step Height Limitations**: Maximum height the robot can step over
- **Step Width Constraints**: Lateral distance between feet during walking
- **Balance Preservation**: Maintaining center of mass within support polygon
- **Gait Pattern Considerations**: Different walking patterns for various speeds

### Humanoid-Aware Path Planning

Nav2 can be adapted for humanoid robots through:

- **Custom Costmap Layers**: Adding humanoid-specific constraints
- **Footstep Planning**: Pre-planning where feet should be placed
- **Balance-Aware Local Planning**: Ensuring stability during navigation
- **Bipedal Controller Plugins**: Specialized controllers for legged locomotion

### Configuration Parameters

Key parameters for humanoid navigation include:

- `max_step_height`: Maximum obstacle height the robot can step over
- `foot_separation`: Distance between feet when standing
- `balance_margin`: Safety margin for balance maintenance
- `gait_pattern`: Walking pattern to use (walk, trot, pace)

## Preparing Robots for Autonomous Movement

Successful navigation requires careful preparation of both the robot and the navigation system:

### Robot Preparation

- **Sensor Configuration**: Proper placement and calibration of navigation sensors
- **Kinematic Calibration**: Accurate measurement of robot dimensions and joint limits
- **Balance Controller Tuning**: Properly tuned controllers for stable locomotion
- **Localization Setup**: Reliable position estimation system

### Environment Preparation

- **Map Quality**: High-quality maps with clear landmarks and navigation paths
- **Calibration Targets**: Visual or geometric features for improved localization
- **Safety Zones**: Designated areas where the robot can safely stop if needed
- **Pathway Clearing**: Ensuring planned paths are clear of obstacles

## Practical Example: Humanoid Navigation Setup

Here's an example of configuring Nav2 for a humanoid robot:

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the behavior tree file is located
    default_nav_to_pose_bt_xml: "humanoid_navigate_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "humanoid_navigate_w_replanning_and_recovery.xml"

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid Follow Path Controller
    FollowPath:
      plugin: "nav2_mppi_controller::Controller"
      # Humanoid-specific parameters
      max_speed: 0.3  # Slower for stability
      min_speed: 0.0
      max_linear_accel: 0.2
      max_linear_decel: -0.2
      max_angular_accel: 0.5
      max_angular_vel: 0.5
      min_angular_vel: 0.1
      speed_scaling_tolerance: 0.1
      # Bipedal-specific parameters
      step_height_limit: 0.15  # 15cm step height
      balance_margin: 0.1      # 10cm balance safety margin

local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: "map"
    robot_base_frame: "base_link"
    use_sim_time: True
    rolling_window: true
    width: 6
    height: 6
    resolution: 0.05
    # Humanoid-specific costmap inflation
    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.5  # Account for humanoid step constraints
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

global_costmap:
  ros__parameters:
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: "map"
    robot_base_frame: "base_link"
    use_sim_time: True
    robot_radius: 0.3  # Humanoid-specific radius
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.6  # Account for humanoid dimensions and step constraints
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific path planning parameters
      step_height_tolerance: 0.15  # Maximum step height
      balance_preservation_weight: 1.0  # Weight for balance preservation

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1
```

### Launch File for Humanoid Navigation

```python
# humanoid_navigation_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')

    # Navigation container
    nav2_container = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[{'autostart': autostart}],
        condition=IfCondition(use_composition)
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('~/progress', '/navigation/progress')],
        condition=IfCondition(use_composition)
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
            ('velocity_smoother/velocity_command', 'cmd_vel')
        ],
        condition=IfCondition(use_composition)
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom')
        ],
        condition=IfCondition(use_composition)
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['controller_server',
                                 'planner_server',
                                 'bt_navigator',
                                 'local_costmap',
                                 'global_costmap',
                                 'amcl']}]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start the navigation system'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_nav2_config'),
                'config',
                'humanoid_nav2_params.yaml'
            ]),
            description='Full path to the navigation parameters file'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use composed bringup if True'),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
            description='Name of container that nodes will load in if use composition'),

        # Nodes
        nav2_container,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager
    ])
```

## Troubleshooting Nav2 Humanoid Setup

### Common Issues and Solutions

1. **Path Planning Failures**
   - Verify map quality and resolution
   - Check costmap inflation parameters
   - Ensure step height constraints are properly configured

2. **Localization Problems**
   - Confirm sensor calibration and placement
   - Verify initial pose estimation
   - Check for sufficient landmarks in the environment

3. **Navigation Oscillation**
   - Adjust controller parameters for humanoid stability
   - Check local planner frequency and control rate
   - Verify robot footprint and collision geometry

### Performance Optimization Tips

- Use appropriate costmap resolution for humanoid dimensions
- Adjust inflation radius to account for step constraints
- Fine-tune controller parameters for stable locomotion
- Monitor navigation performance metrics for optimization

## Summary

This chapter has covered navigation with Nav2 specifically adapted for humanoid robots. You've learned about path planning fundamentals, humanoid-specific navigation constraints, and how to prepare robots for autonomous movement. With this knowledge, you now have a complete understanding of NVIDIA Isaac AI for Humanoid Robots, covering simulation, perception, and navigation capabilities that form the AI brain of modern humanoid robots.