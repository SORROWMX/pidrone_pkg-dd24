# MSP Communication Testing Guide

This guide provides instructions for testing and troubleshooting the MSP communication with the flight controller.

## Quick Test

To quickly test if the MSP communication is working properly:

```bash
roslaunch pidrone_pkg msp_minimal_test.launch
```

This launches only the `msp_offboard.py` node, which attempts to connect to the flight controller board. Check the output for successful connection messages.

## Full System Test

To test the full system with all required nodes:

```bash
roslaunch pidrone_pkg msp_offboard_only.launch
```

## Troubleshooting

If you encounter "Didn't get valid header" errors:

1. Make sure no other nodes are trying to communicate with the flight controller
   - Ensure `flight_controller_node.py` is not running
   - Check for other processes that might be accessing the serial port

2. Check physical connections
   - Ensure the USB cable is properly connected
   - Try unplugging and re-plugging the USB cable

3. Check permissions
   - Make sure your user has permissions to access the serial port
   - Run `sudo chmod a+rw /dev/ttyACM0` (or ttyACM1) to set permissions

4. If using a VM, ensure the USB device is passed through to the VM

5. Check the flight controller firmware
   - Make sure the flight controller is running firmware compatible with MSP protocol

## Comparing with flight_controller_node.py

If `msp_offboard.py` still fails but `flight_controller_node.py` works, you can test the original node:

```bash
roslaunch pidrone_pkg flight_controller_only.launch
```

Compare the behavior and logs between the two approaches to help identify the issue.

## Advanced Debugging

For advanced debugging, you can monitor the raw serial communication:

```bash
sudo apt install setserial
sudo stty -F /dev/ttyACM0 raw
sudo cat /dev/ttyACM0 | hexdump -C
```

This will show the raw bytes being transmitted to and from the flight controller. 