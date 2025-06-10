#!/usr/bin/env python
import sys
import os
import traceback

def update_command_values(mapping):
    """
    Update the command_values.py file based on the determined channel mapping
    
    Args:
        mapping: A list of indices representing the correct order of [roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4]
                For example, if throttle is actually at index 2 instead of 3, mapping would be [0, 1, 3, 2, 4, 5, 6, 7]
    """
    try:
        # Read the current command_values.py file
        with open('command_values.py', 'r') as f:
            lines = f.readlines()
        
        # Parse the current commands
        disarm_cmd = None
        arm_cmd = None
        idle_cmd = None
        
        for line in lines:
            if 'disarm_cmd' in line and '=' in line:
                disarm_cmd = eval(line.split('=')[1].strip())
            elif 'arm_cmd' in line and '=' in line:
                arm_cmd = eval(line.split('=')[1].strip())
            elif 'idle_cmd' in line and '=' in line:
                idle_cmd = eval(line.split('=')[1].strip())
        
        if not all([disarm_cmd, arm_cmd, idle_cmd]):
            print("Failed to parse command values from command_values.py")
            return False
            
        # Remap the commands based on the provided mapping
        new_disarm_cmd = [disarm_cmd[i] for i in mapping]
        new_arm_cmd = [arm_cmd[i] for i in mapping]
        new_idle_cmd = [idle_cmd[i] for i in mapping]
        
        # Create backup of original file
        backup_file = 'command_values.py.bak'
        with open(backup_file, 'w') as f:
            f.writelines(lines)
        
        # Write the new command values
        with open('command_values.py', 'w') as f:
            f.write("# [r,p,y,t]\n")
            f.write(f"disarm_cmd =  {new_disarm_cmd}\n")
            f.write(f"arm_cmd = {new_arm_cmd}\n")
            f.write(f"idle_cmd = {new_idle_cmd}\n")
        
        print(f"Successfully updated command_values.py")
        print(f"Original file backed up to {backup_file}")
        print("\nOld command values:")
        print(f"disarm_cmd = {disarm_cmd}")
        print(f"arm_cmd = {arm_cmd}")
        print(f"idle_cmd = {idle_cmd}")
        print("\nNew command values:")
        print(f"disarm_cmd = {new_disarm_cmd}")
        print(f"arm_cmd = {new_arm_cmd}")
        print(f"idle_cmd = {new_idle_cmd}")
        
        return True
    
    except Exception as e:
        print(f"Error updating command values: {e}")
        print(traceback.format_exc())
        return False

def main():
    print("\n*** COMMAND VALUES FIXER ***")
    print("This script will help you fix the channel mapping in command_values.py")
    print("based on the results from test_rc_channels.py")
    
    print("\nBased on your test results, enter the correct index mapping.")
    print("For example, if your throttle is actually at position 2 instead of 3,")
    print("your mapping would be: 0,1,3,2,4,5,6,7")
    print("\nDefault mapping is: 0,1,2,3,4,5,6,7")
    print("This represents: [roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4]")
    
    try:
        mapping_input = input("\nEnter your mapping (comma-separated indices): ")
        mapping = [int(x.strip()) for x in mapping_input.split(',')]
        
        if len(mapping) != 8:
            print("Error: Mapping must contain exactly 8 values")
            return
            
        if set(mapping) != set(range(8)):
            print("Error: Mapping must contain each index from 0-7 exactly once")
            return
            
        success = update_command_values(mapping)
        
        if success:
            print("\nCommand values updated successfully!")
            print("Please test your drone with the new values (with props removed)")
        else:
            print("\nFailed to update command values")
            
    except ValueError:
        print("Error: Invalid input. Please enter comma-separated integers")
    except Exception as e:
        print(f"Unexpected error: {e}")
        print(traceback.format_exc())

if __name__ == "__main__":
    main() 