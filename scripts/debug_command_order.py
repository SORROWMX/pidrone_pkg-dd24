#!/usr/bin/env python

import time
import sys
import argparse
from h2rMultiWii import MultiWii
import command_values as cmds

def print_command(cmd_name, cmd_values):
    """Print command values in a readable format"""
    print("\n----- {} -----".format(cmd_name))
    print("[0] Roll:     {}".format(cmd_values[0]))
    print("[1] Pitch:    {}".format(cmd_values[1]))
    print("[2] Yaw:      {}".format(cmd_values[2]))
    print("[3] Throttle: {}".format(cmd_values[3]))
    print("[4] AUX1:     {}".format(cmd_values[4]))
    print("[5] AUX2:     {}".format(cmd_values[5]))
    print("[6] AUX3:     {}".format(cmd_values[6]))
    print("[7] AUX4:     {}".format(cmd_values[7]))
    print("------------------")

def main():
    parser = argparse.ArgumentParser(description='Debug MSP command order')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port for the flight controller')
    parser.add_argument('--test', choices=['standard', 'swap_yaw_throttle', 'swap_roll_pitch', 'all_permutations'], 
                       default='standard', help='Test to run')
    
    args = parser.parse_args()
    
    # Connect to the flight controller
    try:
        print("Connecting to flight controller on {}...".format(args.port))
        board = MultiWii(args.port)
        print("Connected!")
    except Exception as e:
        print("Failed to connect: {}".format(e))
        sys.exit(1)
    
    # First, read current RC values to see what we're getting
    print("\nReading current RC values...")
    rc_data = board.getData(MultiWii.RC)
    if rc_data:
        print("\n----- CURRENT RC CHANNELS -----")
        for key, value in rc_data.items():
            if key not in ['elapsed', 'timestamp', 'cmd']:
                print("{}: {}".format(key, value))
        print("----------------------")
    else:
        print("Failed to read RC data!")
    
    # Define test cases
    if args.test == 'standard':
        # Standard order: [roll, pitch, yaw, throttle, aux1, ...]
        test_commands = [
            {
                'name': 'ROLL_TEST',
                'command': [1700, 1500, 1500, 1500, 1900, 1000, 1000, 1000]  # Roll right
            },
            {
                'name': 'PITCH_TEST',
                'command': [1500, 1700, 1500, 1500, 1900, 1000, 1000, 1000]  # Pitch forward
            },
            {
                'name': 'YAW_TEST',
                'command': [1500, 1500, 1700, 1500, 1900, 1000, 1000, 1000]  # Yaw right
            },
            {
                'name': 'THROTTLE_TEST',
                'command': [1500, 1500, 1500, 1700, 1900, 1000, 1000, 1000]  # Increase throttle
            }
        ]
    elif args.test == 'swap_yaw_throttle':
        # Swapped yaw and throttle: [roll, pitch, throttle, yaw, aux1, ...]
        test_commands = [
            {
                'name': 'ROLL_TEST',
                'command': [1700, 1500, 1500, 1500, 1900, 1000, 1000, 1000]  # Roll right
            },
            {
                'name': 'PITCH_TEST',
                'command': [1500, 1700, 1500, 1500, 1900, 1000, 1000, 1000]  # Pitch forward
            },
            {
                'name': 'THROTTLE_TEST (at index 2)',
                'command': [1500, 1500, 1700, 1500, 1900, 1000, 1000, 1000]  # Increase throttle at index 2
            },
            {
                'name': 'YAW_TEST (at index 3)',
                'command': [1500, 1500, 1500, 1700, 1900, 1000, 1000, 1000]  # Yaw right at index 3
            }
        ]
    elif args.test == 'swap_roll_pitch':
        # Swapped roll and pitch: [pitch, roll, yaw, throttle, aux1, ...]
        test_commands = [
            {
                'name': 'PITCH_TEST (at index 0)',
                'command': [1700, 1500, 1500, 1500, 1900, 1000, 1000, 1000]  # Pitch forward at index 0
            },
            {
                'name': 'ROLL_TEST (at index 1)',
                'command': [1500, 1700, 1500, 1500, 1900, 1000, 1000, 1000]  # Roll right at index 1
            },
            {
                'name': 'YAW_TEST',
                'command': [1500, 1500, 1700, 1500, 1900, 1000, 1000, 1000]  # Yaw right
            },
            {
                'name': 'THROTTLE_TEST',
                'command': [1500, 1500, 1500, 1700, 1900, 1000, 1000, 1000]  # Increase throttle
            }
        ]
    else:  # all_permutations - just test one channel at a time
        test_commands = [
            {
                'name': 'INDEX_0_TEST',
                'command': [1700, 1500, 1500, 1500, 1900, 1000, 1000, 1000]  # Value at index 0
            },
            {
                'name': 'INDEX_1_TEST',
                'command': [1500, 1700, 1500, 1500, 1900, 1000, 1000, 1000]  # Value at index 1
            },
            {
                'name': 'INDEX_2_TEST',
                'command': [1500, 1500, 1700, 1500, 1900, 1000, 1000, 1000]  # Value at index 2
            },
            {
                'name': 'INDEX_3_TEST',
                'command': [1500, 1500, 1500, 1700, 1900, 1000, 1000, 1000]  # Value at index 3
            }
        ]
    
    # Run the tests
    print("\nStarting command order tests...")
    
    for test in test_commands:
        print("\n=================================")
        print("Testing: {}".format(test['name']))
        print("=================================")
        
        # Print and send the command
        print_command(test['name'], test['command'])
        print("Sending command...")
        board.send_raw_command(8, MultiWii.SET_RAW_RC, test['command'])
        board.receiveDataPacket()
        print("Command sent!")
        
        # Wait a moment
        time.sleep(1)
        
        # Read back RC values
        rc_data = board.getData(MultiWii.RC)
        if rc_data:
            print("\n----- RC CHANNELS AFTER COMMAND -----")
            for key, value in rc_data.items():
                if key not in ['elapsed', 'timestamp', 'cmd']:
                    print("{}: {}".format(key, value))
            print("----------------------")
        
        # Read attitude
        attitude = board.getData(MultiWii.ATTITUDE)
        if attitude:
            print("\n----- ATTITUDE -----")
            print("Roll:  {:.2f}°".format(attitude['angx']))
            print("Pitch: {:.2f}°".format(attitude['angy']))
            print("Yaw:   {:.2f}°".format(attitude['heading']))
            print("--------------------")
        
        # Return to center before next test
        print("\nReturning to center position...")
        center_command = [1500, 1500, 1500, 1500, 1900, 1000, 1000, 1000]
        board.send_raw_command(8, MultiWii.SET_RAW_RC, center_command)
        board.receiveDataPacket()
        time.sleep(2)  # Give it time to center
    
    # Disarm at the end
    print("\nDisarming...")
    board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
    board.receiveDataPacket()
    print("Disarmed!")
    
    print("\nTest completed!")

if __name__ == "__main__":
    main() 