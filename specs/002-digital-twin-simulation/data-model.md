# Data Model: Digital Twin Simulation for Humanoid Robots

## Core Entities

### Digital Twin Environment
- **Name**: String identifier for the environment
- **Description**: Text description of the simulation environment
- **Physics Parameters**:
  - Gravity (Vector3): x, y, z components of gravitational acceleration
  - Time Step (Float): Simulation time step in seconds
  - Real Time Factor (Float): Multiplier for real-time simulation speed
- **Visual Assets**: Reference to Unity scene assets
- **Environment Objects**: Collection of static and dynamic objects in the environment

### Simulated Robot
- **Robot ID**: Unique identifier for the robot instance
- **Model Reference**: Path to URDF/XACRO model definition
- **Physical Properties**:
  - Mass (Float): Total robot mass in kg
  - Dimensions (Vector3): Bounding box dimensions
  - Inertia Tensor (Matrix3x3): Inertial properties
- **Pose**: Current position and orientation (x, y, z, roll, pitch, yaw)
- **Joint States**: Collection of joint positions, velocities, efforts
- **Attached Sensors**: Collection of sensor instances

### Simulated Sensors
- **Sensor ID**: Unique identifier for the sensor instance
- **Type**: Enum (LIDAR, DepthCamera, IMU, etc.)
- **Parent Robot**: Reference to the robot this sensor is attached to
- **Mounting Pose**: Position and orientation relative to robot
- **Parameters**: Type-specific configuration parameters
- **Output Topic**: ROS 2 topic name for sensor data

### Training Scenario
- **Scenario ID**: Unique identifier for the training scenario
- **Title**: Human-readable title
- **Description**: Detailed description of the scenario
- **Environment**: Reference to Digital Twin Environment
- **Initial Robot State**: Starting pose and joint configuration
- **Objectives**: Collection of learning objectives
- **Success Criteria**: Conditions that define scenario completion

## Relationships

### Digital Twin Environment contains:
- Multiple Simulated Robots
- Multiple Environment Objects
- Multiple Training Scenarios

### Simulated Robot has:
- One-to-many relationship with Simulated Sensors
- Belongs to one Digital Twin Environment
- Can be part of multiple Training Scenarios

### Training Scenario uses:
- One Digital Twin Environment
- One initial Simulated Robot state
- Multiple success criteria that reference robot states or sensor data

## State Transitions

### Simulated Robot States:
1. **Idle**: Robot is initialized but not moving
2. **Moving**: Robot is executing motion commands
3. **Sensing**: Robot is actively collecting sensor data
4. **Collision**: Robot has collided with environment
5. **Paused**: Simulation is paused for this robot
6. **Completed**: Robot has achieved scenario objectives

### Training Scenario States:
1. **Not Started**: Scenario is available but not initiated
2. **In Progress**: Scenario is currently running
3. **Completed**: Scenario objectives have been met
4. **Failed**: Scenario has ended without meeting objectives

## Validation Rules

### Digital Twin Environment:
- Physics parameters must be within physically realistic ranges
- Environment objects must not overlap initially
- Time step must be positive and reasonable for stability

### Simulated Robot:
- Joint positions must be within physical limits defined in URDF
- Robot must be placed within environment boundaries
- Mass must be positive and realistic for humanoid robot

### Simulated Sensors:
- Mounting pose must be physically possible on the robot
- Output topic names must follow ROS 2 naming conventions
- Sensor parameters must be valid for the sensor type

### Training Scenario:
- Initial robot state must be valid within the environment
- Success criteria must reference measurable quantities
- Scenario must have at least one objective