#!/usr/bin/env python

import time
import sys
import argparse
import json
import os
from h2rMultiWii import MultiWii
import command_values as cmds

# Добавляем поддержку ROS, если он доступен
try:
    import rospy
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS not available, running without ROS support")

def print_command(cmd_name, cmd_values):
    """Print command values in a readable format"""
    print("\n----- {} -----".format(cmd_name))
    print("Roll:     {}".format(cmd_values[0]))
    print("Pitch:    {}".format(cmd_values[1]))
    print("Yaw:      {}".format(cmd_values[2]))
    print("Throttle: {}".format(cmd_values[3]))
    print("AUX1:     {}".format(cmd_values[4]))
    print("AUX2:     {}".format(cmd_values[5]))
    print("AUX3:     {}".format(cmd_values[6]))
    print("AUX4:     {}".format(cmd_values[7]))
    print("------------------")

class MSPCommandDebugger:
    def __init__(self, port='/dev/ttyACM0', log_to_file=False):
        self.port = port
        self.board = None
        self.log_to_file = log_to_file
        self.log_file = None
        self.data_log = []
        
        # ROS publishers
        self.debug_pub = None
        self.log_pub = None
        
        # Connect to the board
        self.connect()
        
        # Initialize log file if needed
        if self.log_to_file:
            self._init_log_file()
    
    def connect(self):
        """Connect to the flight controller board"""
        try:
            print("Connecting to flight controller on {}...".format(self.port))
            self.board = MultiWii(self.port)
            print("Connected!")
            return True
        except Exception as e:
            print("Failed to connect: {}".format(e))
            return False
    
    def _init_log_file(self):
        """Initialize log file with timestamp in name"""
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_file = os.path.join(log_dir, f"msp_commands_{timestamp}.log")
        
        with open(self.log_file, 'w') as f:
            f.write("# MSP Commands Debug Log\n")
            f.write(f"# Started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("# Format: timestamp,type,data\n")
        
        print(f"Logging to file: {self.log_file}")
    
    def _log_to_file(self, log_type, data):
        """Log data to file"""
        if not self.log_to_file or not self.log_file:
            return
        
        try:
            with open(self.log_file, 'a') as f:
                timestamp = time.time()
                json_data = json.dumps(data)
                f.write(f"{timestamp},{log_type},{json_data}\n")
        except Exception as e:
            print(f"Failed to write to log file: {e}")
    
    def _publish_to_ros(self, data_type, data):
        """Publish data to ROS topic if available"""
        if not ROS_AVAILABLE or not self.debug_pub:
            return
        
        try:
            msg_data = {
                "type": data_type,
                "data": data,
                "timestamp": time.time()
            }
            self.debug_pub.publish(json.dumps(msg_data))
        except Exception as e:
            print(f"Failed to publish to ROS: {e}")
    
    def send_command(self, command, cmd_name="CUSTOM"):
        """Send command to the flight controller and log it"""
        print_command(cmd_name, command)
        
        # Log command to file and ROS
        cmd_data = {
            "name": cmd_name,
            "values": command,
            "timestamp": time.time()
        }
        self._log_to_file("command", cmd_data)
        self._publish_to_ros("command", cmd_data)
        self.data_log.append(cmd_data)
        
        # Send the command
        print("Sending command...")
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, command)
        self.board.receiveDataPacket()
        print("Command sent!")
    
    def monitor_rc_channels(self, duration=10):
        """Monitor RC channels for a specified duration"""
        print(f"\nMonitoring RC channels for {duration} seconds. Press Ctrl+C to stop.")
        
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                # Get RC channel data
                rc_data = self.board.getData(MultiWii.RC)
                if rc_data:
                    # Filter out non-channel data
                    channel_data = {k: v for k, v in rc_data.items() 
                                  if k not in ['elapsed', 'timestamp', 'cmd']}
                    
                    print("\n----- RC CHANNELS -----")
                    for key, value in channel_data.items():
                        print("{}: {}".format(key, value))
                    print("----------------------")
                    
                    # Log to file and ROS
                    self._log_to_file("rc_channels", channel_data)
                    self._publish_to_ros("rc_channels", channel_data)
                    self.data_log.append({"type": "rc_channels", "data": channel_data, "timestamp": time.time()})
                
                # Get attitude data
                attitude = self.board.getData(MultiWii.ATTITUDE)
                if attitude:
                    # Filter out non-attitude data
                    att_data = {k: v for k, v in attitude.items() 
                               if k not in ['elapsed', 'timestamp', 'cmd']}
                    
                    print("\n----- ATTITUDE -----")
                    print("Roll:  {:.2f}°".format(att_data.get('angx', 0)))
                    print("Pitch: {:.2f}°".format(att_data.get('angy', 0)))
                    print("Yaw:   {:.2f}°".format(att_data.get('heading', 0)))
                    print("--------------------")
                    
                    # Log to file and ROS
                    self._log_to_file("attitude", att_data)
                    self._publish_to_ros("attitude", att_data)
                    self.data_log.append({"type": "attitude", "data": att_data, "timestamp": time.time()})
                
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
    
    def save_log_to_file(self, filename=None):
        """Save current debug data to a file"""
        if not filename:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"msp_commands_export_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.data_log, f, indent=2)
            
            print(f"Log saved to {filename}")
            
            # Publish filename to log topic
            if ROS_AVAILABLE and self.log_pub:
                self.log_pub.publish(f"Log saved to {filename}")
            
            return True
        except Exception as e:
            print(f"Failed to save log: {e}")
            return False
    
    def initialize_ros(self):
        """Initialize ROS publishers"""
        if not ROS_AVAILABLE:
            return False
        
        try:
            rospy.init_node('msp_command_debugger', anonymous=True)
            
            self.debug_pub = rospy.Publisher('/pidrone/msp_debug_data', String, queue_size=10)
            self.log_pub = rospy.Publisher('/pidrone/msp_debug_log', String, queue_size=10)
            
            print("ROS publishers initialized")
            return True
        except Exception as e:
            print(f"Failed to initialize ROS: {e}")
            return False
    
    def close(self):
        """Clean up resources"""
        if self.board:
            print("Closing connection...")
            self.board.close()
            print("Connection closed")

def main():
    parser = argparse.ArgumentParser(description='Debug MSP commands')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port for the flight controller')
    parser.add_argument('--command', choices=['arm', 'disarm', 'idle', 'custom'], default='custom', help='Command to send')
    parser.add_argument('--roll', type=int, default=1500, help='Roll value (1000-2000)')
    parser.add_argument('--pitch', type=int, default=1500, help='Pitch value (1000-2000)')
    parser.add_argument('--yaw', type=int, default=1500, help='Yaw value (1000-2000)')
    parser.add_argument('--throttle', type=int, default=1500, help='Throttle value (1000-2000)')
    parser.add_argument('--aux1', type=int, default=1500, help='AUX1 value (1000-2000)')
    parser.add_argument('--monitor', action='store_true', help='Monitor RC channels after sending command')
    parser.add_argument('--duration', type=int, default=10, help='Duration for monitoring (seconds)')
    parser.add_argument('--log-file', action='store_true', help='Log data to file')
    parser.add_argument('--export', type=str, help='Export logs to specified file')
    parser.add_argument('--ros', action='store_true', help='Enable ROS publishing')
    
    args = parser.parse_args()
    
    # Create debugger
    debugger = MSPCommandDebugger(args.port, log_to_file=args.log_file)
    
    try:
        # Initialize ROS if requested
        if args.ros and ROS_AVAILABLE:
            debugger.initialize_ros()
        
        # Determine which command to send
        if args.command == 'arm':
            command = cmds.arm_cmd
            cmd_name = "ARM"
        elif args.command == 'disarm':
            command = cmds.disarm_cmd
            cmd_name = "DISARM"
        elif args.command == 'idle':
            command = cmds.idle_cmd
            cmd_name = "IDLE"
        else:  # custom
            command = [args.roll, args.pitch, args.yaw, args.throttle, args.aux1, 1000, 1000, 1000]
            cmd_name = "CUSTOM"
        
        # Send the command
        debugger.send_command(command, cmd_name)
        
        # Monitor RC channels if requested
        if args.monitor:
            debugger.monitor_rc_channels(args.duration)
        
        # Export logs if requested
        if args.export:
            debugger.save_log_to_file(args.export)
    
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        debugger.close()

if __name__ == "__main__":
    main() 