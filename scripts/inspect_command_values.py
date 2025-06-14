#!/usr/bin/env python

import time
import sys
import os
import argparse
import command_values as cmds

def print_current_commands():
    """Print current command values from command_values.py"""
    print("\n----- CURRENT COMMAND VALUES -----")
    print("DISARM_CMD:")
    print("  Roll:     {}".format(cmds.disarm_cmd[0]))
    print("  Pitch:    {}".format(cmds.disarm_cmd[1]))
    print("  Yaw:      {}".format(cmds.disarm_cmd[2]))
    print("  Throttle: {}".format(cmds.disarm_cmd[3]))
    print("  AUX1:     {}".format(cmds.disarm_cmd[4]))
    print("  AUX2:     {}".format(cmds.disarm_cmd[5]))
    print("  AUX3:     {}".format(cmds.disarm_cmd[6]))
    print("  AUX4:     {}".format(cmds.disarm_cmd[7]))
    
    print("\nARM_CMD:")
    print("  Roll:     {}".format(cmds.arm_cmd[0]))
    print("  Pitch:    {}".format(cmds.arm_cmd[1]))
    print("  Yaw:      {}".format(cmds.arm_cmd[2]))
    print("  Throttle: {}".format(cmds.arm_cmd[3]))
    print("  AUX1:     {}".format(cmds.arm_cmd[4]))
    print("  AUX2:     {}".format(cmds.arm_cmd[5]))
    print("  AUX3:     {}".format(cmds.arm_cmd[6]))
    print("  AUX4:     {}".format(cmds.arm_cmd[7]))
    
    print("\nIDLE_CMD:")
    print("  Roll:     {}".format(cmds.idle_cmd[0]))
    print("  Pitch:    {}".format(cmds.idle_cmd[1]))
    print("  Yaw:      {}".format(cmds.idle_cmd[2]))
    print("  Throttle: {}".format(cmds.idle_cmd[3]))
    print("  AUX1:     {}".format(cmds.idle_cmd[4]))
    print("  AUX2:     {}".format(cmds.idle_cmd[5]))
    print("  AUX3:     {}".format(cmds.idle_cmd[6]))
    print("  AUX4:     {}".format(cmds.idle_cmd[7]))
    print("--------------------------------")

def swap_yaw_throttle():
    """Swap yaw and throttle positions in command_values.py"""
    # Create backup
    backup_file = "command_values.py.bak"
    if not os.path.exists(backup_file):
        with open("scripts/command_values.py", "r") as f:
            content = f.read()
        with open(backup_file, "w") as f:
            f.write(content)
        print("Backup created: {}".format(backup_file))
    
    # Modify command_values.py
    with open("scripts/command_values.py", "r") as f:
        lines = f.readlines()
    
    # Find and modify the command arrays
    modified_lines = []
    for line in lines:
        if "disarm_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            # Swap yaw and throttle (index 2 and 3)
            values[2], values[3] = values[3], values[2]
            modified_lines.append("disarm_cmd = {}\n".format(values))
        elif "arm_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            # Swap yaw and throttle (index 2 and 3)
            values[2], values[3] = values[3], values[2]
            modified_lines.append("arm_cmd = {}\n".format(values))
        elif "idle_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            # Swap yaw and throttle (index 2 and 3)
            values[2], values[3] = values[3], values[2]
            modified_lines.append("idle_cmd = {}\n".format(values))
        else:
            modified_lines.append(line)
    
    # Write modified content back
    with open("scripts/command_values.py", "w") as f:
        f.writelines(modified_lines)
    
    print("Yaw and throttle positions swapped in command_values.py")
    
    # Reload the module
    reload(cmds)

def restore_backup():
    """Restore command_values.py from backup"""
    backup_file = "command_values.py.bak"
    if os.path.exists(backup_file):
        with open(backup_file, "r") as f:
            content = f.read()
        with open("scripts/command_values.py", "w") as f:
            f.write(content)
        print("Restored command_values.py from backup")
        
        # Reload the module
        reload(cmds)
    else:
        print("No backup file found!")

def update_command_values(new_order):
    """Update command_values.py with a new order of channels"""
    # Create backup if it doesn't exist
    backup_file = "command_values.py.bak"
    if not os.path.exists(backup_file):
        with open("scripts/command_values.py", "r") as f:
            content = f.read()
        with open(backup_file, "w") as f:
            f.write(content)
        print("Backup created: {}".format(backup_file))
    
    # Parse the new order
    order_map = {}
    for i, pos in enumerate(new_order):
        order_map[pos] = i
    
    # Standard order is: roll, pitch, yaw, throttle
    standard_order = ['roll', 'pitch', 'yaw', 'throttle']
    
    # Modify command_values.py
    with open("scripts/command_values.py", "r") as f:
        lines = f.readlines()
    
    # Find and modify the command arrays
    modified_lines = []
    for line in lines:
        if "disarm_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            new_values = list(values)  # Create a copy
            # Rearrange the first 4 values according to the new order
            for i, channel in enumerate(standard_order):
                new_values[order_map[channel]] = values[i]
            modified_lines.append("disarm_cmd = {}\n".format(new_values))
        elif "arm_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            new_values = list(values)  # Create a copy
            # Rearrange the first 4 values according to the new order
            for i, channel in enumerate(standard_order):
                new_values[order_map[channel]] = values[i]
            modified_lines.append("arm_cmd = {}\n".format(new_values))
        elif "idle_cmd =" in line:
            values = eval(line.split("=")[1].strip())
            new_values = list(values)  # Create a copy
            # Rearrange the first 4 values according to the new order
            for i, channel in enumerate(standard_order):
                new_values[order_map[channel]] = values[i]
            modified_lines.append("idle_cmd = {}\n".format(new_values))
        else:
            modified_lines.append(line)
    
    # Write modified content back
    with open("scripts/command_values.py", "w") as f:
        f.writelines(modified_lines)
    
    print("Command values updated with new order: {}".format(new_order))
    
    # Reload the module
    reload(cmds)

def main():
    parser = argparse.ArgumentParser(description='Inspect and modify command_values.py')
    parser.add_argument('--action', choices=['show', 'swap_yt', 'restore', 'update'], 
                       default='show', help='Action to perform')
    parser.add_argument('--order', nargs=4, default=['roll', 'pitch', 'yaw', 'throttle'],
                       help='New order of channels (roll, pitch, yaw, throttle)')
    
    args = parser.parse_args()
    
    if args.action == 'show':
        print_current_commands()
    elif args.action == 'swap_yt':
        swap_yaw_throttle()
        print_current_commands()
    elif args.action == 'restore':
        restore_backup()
        print_current_commands()
    elif args.action == 'update':
        update_command_values(args.order)
        print_current_commands()

if __name__ == "__main__":
    main() 