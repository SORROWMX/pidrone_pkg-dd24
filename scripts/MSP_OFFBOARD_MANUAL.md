# MSP Offboard Manual

## System Overview

MSP Offboard is a high-level drone control system using the MSP protocol, similar to simple_offboard in PX4. The system provides:

- Position control in 3D space
- Velocity control
- Automatic takeoff and landing
- Flight to specified coordinates
- Telemetry access

## System Components

The system consists of the following components:

1. **msp_offboard.py** - main control node
2. **msp_offboard_msgs.py** - message definitions and service handlers
3. **msp_offboard.launch** - launch file including:
   - tof.launch - Time-of-Flight sensor
   - raspicam_node.launch - camera
   - flight_controller_node.py - flight controller interface
   - pid_controller.py - PID controller
   - state_estimator.py - drone state estimation
   - optical_flow_node.py - optical flow processing
   - rigid_transform_node.py - coordinate transformations
   - rosbridge_server - web interface
   - web_video_server - video streaming

## System Launch

To launch MSP Offboard, use the command:

```bash
roslaunch pidrone_pkg msp_offboard.launch
```

## Main Functions and Services

### Takeoff

```bash
rosservice call /pidrone/msp_offboard/takeoff
```

The drone will smoothly take off to the default height (0.3 meters).

### Landing

```bash
rosservice call /pidrone/msp_offboard/land
```

The drone will perform a smooth landing.

### Navigate to Point

```bash
rosservice call /pidrone/msp_offboard/navigate "x 0.5 y 0.5 z 0.3 speed 0.5 frame_id map"
```

The drone will move to the specified point at the given speed. Parameters:

- **x, y, z** - target point coordinates (in meters)
- **speed** - flight speed (in m/s)
- **frame_id** - coordinate system ("map" for absolute coordinates, "body" for relative)
- **auto_arm** - automatic takeoff if the drone is not flying (true/false)

Example with automatic takeoff:

```bash
rosservice call /pidrone/msp_offboard/navigate "x 0.5 y 0.5 z 0.3 speed 0.5 frame_id map auto_arm true"
```

### Velocity Control

```bash
rosservice call /pidrone/msp_offboard/set_velocity "vx 0.2 vy 0.0 vz 0.0 yaw_rate 0.0"
```

Parameters:

- **vx, vy, vz** - velocity components along X, Y, Z axes (in m/s)
- **yaw_rate** - yaw angular velocity (in rad/s)
- **frame_id** - coordinate system ("body" by default)

### Position Control

```bash
rosservice call /pidrone/msp_offboard/set_position "x 0.5 y 0.5 z 0.3 frame_id map"
```

Parameters:

- **x, y, z** - target point coordinates (in meters)
- **frame_id** - coordinate system ("map" for absolute coordinates, "body" for relative)

### Get Telemetry

```bash
rosservice call /pidrone/msp_offboard/get_telemetry
```

Returns the current drone state, including:
- Position (x, y, z)
- Velocity (vx, vy, vz)
- Orientation (roll, pitch, yaw)
- Status (armed, flying, mode)

## System Parameters

- **default_speed** - default flight speed (0.5 m/s)
- **default_takeoff_height** - default takeoff height (0.3 m)
- **height_tolerance** - acceptable height error (0.05 m)
- **position_tolerance** - acceptable position error (0.1 m)

## Control Modes

MSP Offboard supports two control modes:

1. **position** - position control
2. **velocity** - velocity control

## Safety Limitations

The system has built-in safety limitations:

- Maximum flight height: 0.5 m
- Minimum flight height: 0.05 m
- Maximum speed: 0.5 m/s horizontally, 0.3 m/s vertically
- Adaptive correction when unable to reach target point

## Component Interaction

1. **msp_offboard.py** receives commands through ROS services
2. **flight_controller_node.py** sends commands to the flight controller via MSP protocol
3. **state_estimator.py** estimates the current drone position
4. **optical_flow_node.py** processes camera data to determine velocity
5. **rigid_transform_node.py** transforms coordinates between different reference frames

## Implementation Features

### Smooth Movement

When calling the navigate function, the drone moves to the target point smoothly by breaking the trajectory into several intermediate points. This provides more stable flight and better positioning accuracy.

### Adaptive Logic

The system uses adaptive logic during takeoff and movement:
- If the drone cannot reach the specified height or position, the system automatically adjusts the target point
- When exceeding height, the system reduces the target height instead of emergency disarm

### Automatic Takeoff

The navigate function supports the auto_arm parameter, which allows automatic takeoff if the drone is not yet in the air.

## Monitoring and Debugging

The system publishes the following topics for monitoring:

- **/pidrone/state** - current drone state
- **/pidrone/height_stable** - height stability flag
- **/pidrone/heartbeat/msp_offboard** - system activity signal

## Usage Examples

### Takeoff, Movement, and Landing

```bash
# Takeoff
rosservice call /pidrone/msp_offboard/takeoff

# Wait for stabilization
sleep 3

# Move to point
rosservice call /pidrone/msp_offboard/navigate "x 0.5 y 0.0 z 0.3 speed 0.3 frame_id map"

# Wait for movement completion
sleep 5

# Return to starting point
rosservice call /pidrone/msp_offboard/navigate "x 0.0 y 0.0 z 0.3 speed 0.3 frame_id map"

# Wait for movement completion
sleep 5

# Landing
rosservice call /pidrone/msp_offboard/land
```

## Troubleshooting

### Drone Won't Take Off

- Check battery level
- Make sure all ROS nodes are running
- Check for errors in logs

### Drone is Unstable in Flight

- Check optical flow operation
- Make sure the surface under the drone has enough texture
- Reduce movement speed

### Drone Doesn't Reach Target Position

- Check that the target position is within the allowed range
- Increase position_tolerance
- Check state_estimator operation

## Diagnostics via ROS

Use the following commands for system diagnostics:

```bash
# View current state
rosservice call /pidrone/msp_offboard/get_telemetry

# View topics
rostopic list | grep pidrone

# View camera data
rosrun image_view image_view image:=/raspicam_node/image

# View ToF sensor data
rostopic echo /pidrone/range
```

## Extending Functionality

MSP Offboard is designed to be extensible. You can add new functions by following these steps:

1. Add a new method to the MSPOffboard class in msp_offboard.py
2. Define corresponding request and response classes in msp_offboard_msgs.py
3. Create a service handler in msp_offboard_msgs.py
4. Register the new service in the __init__ method of the MSPOffboard class

MSP Offboard provides a convenient high-level interface for controlling a drone via MSP protocol. The system integrates various components to ensure stable flight and accurate positioning in space. 