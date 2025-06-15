# PiDrone Launch Options

This README explains the different launch options available for the PiDrone package.

## Communication with the Flight Controller

The PiDrone package has two different nodes that can communicate with the flight controller board:

1. `flight_controller_node.py` - The original node that communicates with the flight controller, reads IMU data, and sends basic control commands.
2. `msp_offboard.py` - A more advanced control node that provides position control, velocity control, and other features.

## Launch Options

### 1. Using the flight_controller_node.py only

```bash
roslaunch pidrone_pkg flight_controller_only.launch
```

This launches the basic flight controller node without the MSP Offboard functionality.

### 2. Using the msp_offboard.py only

```bash
roslaunch pidrone_pkg msp_offboard_only.launch
```

This launches the MSP Offboard node for advanced control without the standard flight controller node.

### 3. Original launch (PROBLEMATIC - DO NOT USE!)

```bash
roslaunch pidrone_pkg msp_offboard.launch
```

⚠️ **WARNING**: This launch file attempts to start both `flight_controller_node.py` and `msp_offboard.py`, which will cause errors because both try to connect to the same serial port. You will see "Didn't get valid header" errors and other communication failures.

## Troubleshooting

If you see errors like these:
```
"Didn't get valid header: "
"Failed to get data after 3 attempts"
"Exception in receiveDataPacket: device reports readiness to read but returned no data (device disconnected or multiple access on port?)"
```

It means multiple nodes are trying to connect to the same serial port. Use one of the first two launch options above instead.

## Choosing Which Mode to Use

- Use `flight_controller_only.launch` for basic manual control and testing.
- Use `msp_offboard_only.launch` for advanced features like position control, waypoint navigation, etc. 