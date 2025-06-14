#!/usr/bin/env python

import time
import sys
import os
import argparse
import signal
import logging
import json
from threading import Thread
import numpy as np
import rospy
from h2rMultiWii import MultiWii
import command_values as cmds
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Empty, Bool, String
from geometry_msgs.msg import Quaternion
from pidrone_pkg.msg import Mode, RC, State

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('fc_debug')

class FlightControllerDebugger:
    """
    Debug tool for analyzing the flight controller's behavior and MSP communication
    """
    def __init__(self, port='/dev/ttyACM0', log_to_file=False):
        self.port = port
        self.board = None
        self.running = False
        self.command = cmds.disarm_cmd.copy()
        self.log_to_file = log_to_file
        self.log_file = None
        
        # Tracking variables
        self.last_attitude = None
        self.last_raw_imu = None
        self.last_rc = None
        self.command_history = []
        self.mode = "DISARMED"
        
        # ROS publishers (if ROS is initialized)
        self.imu_pub = None
        self.mode_pub = None
        self.rc_pub = None
        self.debug_pub = None
        self.log_pub = None
        
        # Connect to the flight controller
        self.connect()
        
        # Initialize log file if needed
        if self.log_to_file:
            self._init_log_file()
    
    def _init_log_file(self):
        """Initialize log file with timestamp in name"""
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_file = os.path.join(log_dir, f"fc_debug_{timestamp}.log")
        
        with open(self.log_file, 'w') as f:
            f.write("# Flight Controller Debug Log\n")
            f.write(f"# Started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("# Format: timestamp,type,data\n")
        
        logger.info(f"Logging to file: {self.log_file}")
    
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
            logger.error(f"Failed to write to log file: {e}")
    
    def connect(self):
        """Connect to the flight controller board"""
        try:
            logger.info("Connecting to flight controller on %s...", self.port)
            self.board = MultiWii(self.port)
            logger.info("Connected successfully!")
            return True
        except Exception as e:
            logger.error("Failed to connect: %s", e)
            return False
    
    def start_monitoring(self):
        """Start monitoring the flight controller in a separate thread"""
        if self.board is None:
            logger.error("Not connected to flight controller")
            return False
        
        self.running = True
        self.monitor_thread = Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        logger.info("Monitoring started")
        return True
    
    def stop_monitoring(self):
        """Stop the monitoring thread"""
        self.running = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=2.0)
        logger.info("Monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop that runs in a separate thread"""
        rate = 0.05  # 20Hz
        while self.running:
            try:
                # Get attitude data
                attitude = self.board.getData(MultiWii.ATTITUDE)
                if attitude:
                    self.last_attitude = attitude
                    self._process_attitude(attitude)
                
                # Get raw IMU data
                raw_imu = self.board.getData(MultiWii.RAW_IMU)
                if raw_imu:
                    self.last_raw_imu = raw_imu
                    self._process_raw_imu(raw_imu)
                
                # Get RC data
                rc = self.board.getData(MultiWii.RC)
                if rc:
                    self.last_rc = rc
                    self._process_rc(rc)
                
                # Sleep to maintain the rate
                time.sleep(rate)
            except Exception as e:
                logger.error("Error in monitor loop: %s", e)
                time.sleep(1)  # Sleep longer on error
    
    def _process_attitude(self, attitude):
        """Process attitude data"""
        logger.debug("Attitude: Roll=%.2f° Pitch=%.2f° Yaw=%.2f°", 
                    attitude['angx'], attitude['angy'], attitude['heading'])
        
        # Log to file
        self._log_to_file("attitude", attitude)
        
        # Publish to ROS if initialized
        if self.imu_pub:
            self._publish_imu_data(attitude, self.last_raw_imu)
        
        # Publish to debug topic
        if self.debug_pub:
            debug_data = {
                "type": "attitude",
                "roll": attitude['angx'],
                "pitch": attitude['angy'],
                "yaw": attitude['heading'],
                "timestamp": time.time()
            }
            self.debug_pub.publish(json.dumps(debug_data))
    
    def _process_raw_imu(self, raw_imu):
        """Process raw IMU data"""
        logger.debug("Raw IMU: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d", 
                    raw_imu['ax'], raw_imu['ay'], raw_imu['az'],
                    raw_imu['gx'], raw_imu['gy'], raw_imu['gz'])
        
        # Log to file
        self._log_to_file("raw_imu", raw_imu)
        
        # Publish to ROS if initialized and we have attitude data
        if self.imu_pub and self.last_attitude:
            self._publish_imu_data(self.last_attitude, raw_imu)
        
        # Publish to debug topic
        if self.debug_pub:
            debug_data = {
                "type": "raw_imu",
                "ax": raw_imu['ax'],
                "ay": raw_imu['ay'],
                "az": raw_imu['az'],
                "gx": raw_imu['gx'],
                "gy": raw_imu['gy'],
                "gz": raw_imu['gz'],
                "timestamp": time.time()
            }
            self.debug_pub.publish(json.dumps(debug_data))
    
    def _process_rc(self, rc):
        """Process RC channel data"""
        rc_values = []
        for key in ['roll', 'pitch', 'yaw', 'throttle']:
            if key in rc:
                rc_values.append(rc[key])
        
        if len(rc_values) >= 4:
            logger.debug("RC: Roll=%d Pitch=%d Yaw=%d Throttle=%d", 
                        rc_values[0], rc_values[1], rc_values[2], rc_values[3])
            
            # Log to file
            self._log_to_file("rc", rc)
            
            # Publish to ROS if initialized
            if self.rc_pub:
                self._publish_rc_data(rc)
            
            # Publish to debug topic
            if self.debug_pub:
                debug_data = {
                    "type": "rc",
                    "roll": rc.get('roll', 1500),
                    "pitch": rc.get('pitch', 1500),
                    "yaw": rc.get('yaw', 1500),
                    "throttle": rc.get('throttle', 1500),
                    "timestamp": time.time()
                }
                self.debug_pub.publish(json.dumps(debug_data))
    
    def _publish_imu_data(self, attitude, raw_imu):
        """Publish IMU data to ROS (similar to flight_controller_node.py)"""
        if not (attitude and raw_imu):
            return
        
        try:
            import tf
        except ImportError:
            logger.error("Failed to import tf module")
            return
        
        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()
        
        imu_msg = Imu()
        imu_msg.header = header
        
        # Convert attitude to quaternion
        roll = np.deg2rad(attitude['angx'])
        pitch = -np.deg2rad(attitude['angy'])
        heading = np.deg2rad(attitude['heading'])
        heading = (-heading) % (2 * np.pi)
        
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        
        # Set orientation
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Set linear acceleration and angular velocity
        # Note: This is simplified and doesn't include all the processing from flight_controller_node.py
        imu_msg.linear_acceleration.x = raw_imu['ax'] / 512.0 * 9.8
        imu_msg.linear_acceleration.y = raw_imu['ay'] / 512.0 * 9.8
        imu_msg.linear_acceleration.z = raw_imu['az'] / 512.0 * 9.8
        
        self.imu_pub.publish(imu_msg)
    
    def _publish_rc_data(self, rc):
        """Publish RC data to ROS"""
        rc_msg = RC()
        rc_msg.roll = rc.get('roll', 1500)
        rc_msg.pitch = rc.get('pitch', 1500)
        rc_msg.yaw = rc.get('yaw', 1500)
        rc_msg.throttle = rc.get('throttle', 1500)
        
        self.rc_pub.publish(rc_msg)
    
    def send_command(self, command, description=""):
        """Send a command to the flight controller and log it"""
        if self.board is None:
            logger.error("Not connected to flight controller")
            return False
        
        logger.info("Sending command: %s %s", command, description)
        self.command_history.append((command.copy(), description, time.time()))
        
        # Log to file
        self._log_to_file("command", {
            "command": command,
            "description": description,
            "timestamp": time.time()
        })
        
        # Publish to debug topic
        if self.debug_pub:
            debug_data = {
                "type": "command",
                "command": command,
                "description": description,
                "timestamp": time.time()
            }
            self.debug_pub.publish(json.dumps(debug_data))
        
        try:
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, command)
            self.board.receiveDataPacket()
            return True
        except Exception as e:
            logger.error("Failed to send command: %s", e)
            return False
    
    def arm(self):
        """Arm the drone"""
        self.mode = "ARMED"
        return self.send_command(cmds.arm_cmd, "ARM")
    
    def disarm(self):
        """Disarm the drone"""
        self.mode = "DISARMED"
        return self.send_command(cmds.disarm_cmd, "DISARM")
    
    def set_idle(self):
        """Set to idle command"""
        return self.send_command(cmds.idle_cmd, "IDLE")
    
    def test_roll(self, value=1700):
        """Test roll command"""
        cmd = cmds.idle_cmd.copy()
        cmd[0] = value  # Assuming roll is at index 0
        return self.send_command(cmd, "TEST ROLL")
    
    def test_pitch(self, value=1700):
        """Test pitch command"""
        cmd = cmds.idle_cmd.copy()
        cmd[1] = value  # Assuming pitch is at index 1
        return self.send_command(cmd, "TEST PITCH")
    
    def test_yaw(self, value=1700):
        """Test yaw command"""
        cmd = cmds.idle_cmd.copy()
        cmd[2] = value  # Assuming yaw is at index 2
        return self.send_command(cmd, "TEST YAW")
    
    def test_throttle(self, value=1700):
        """Test throttle command"""
        cmd = cmds.idle_cmd.copy()
        cmd[3] = value  # Assuming throttle is at index 3
        return self.send_command(cmd, "TEST THROTTLE")
    
    def test_all_channels(self):
        """Test all channels one by one"""
        logger.info("Testing all channels...")
        
        # Test roll
        self.test_roll()
        time.sleep(1)
        self.set_idle()
        time.sleep(1)
        
        # Test pitch
        self.test_pitch()
        time.sleep(1)
        self.set_idle()
        time.sleep(1)
        
        # Test yaw
        self.test_yaw()
        time.sleep(1)
        self.set_idle()
        time.sleep(1)
        
        # Test throttle
        self.test_throttle()
        time.sleep(1)
        self.set_idle()
        time.sleep(1)
        
        logger.info("All channel tests completed")
    
    def verify_command_order(self):
        """Verify the command order by testing each channel and observing RC feedback"""
        logger.info("Verifying command order...")
        
        # Store original values
        original_rc = None
        while not original_rc:
            original_rc = self.board.getData(MultiWii.RC)
            time.sleep(0.1)
        
        # Test each channel
        channel_names = ["Roll", "Pitch", "Yaw", "Throttle"]
        detected_mapping = {}
        
        for i in range(4):
            # Create a command with only one channel changed
            cmd = cmds.idle_cmd.copy()
            cmd[i] = 1700  # Increase the value for this channel
            
            logger.info("Testing index %d with value 1700", i)
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmd)
            self.board.receiveDataPacket()
            
            # Wait for the change to take effect
            time.sleep(0.5)
            
            # Read RC values
            new_rc = None
            while not new_rc:
                new_rc = self.board.getData(MultiWii.RC)
                time.sleep(0.1)
            
            # Detect which channel changed
            changed_channel = None
            for key in ['roll', 'pitch', 'yaw', 'throttle']:
                if key in new_rc and key in original_rc:
                    diff = abs(new_rc[key] - original_rc[key])
                    if diff > 100:  # Significant change
                        changed_channel = key
                        logger.info("Index %d affects %s (changed by %d)", i, key, diff)
                        detected_mapping[i] = key
                        break
            
            if not changed_channel:
                logger.warning("No significant change detected for index %d", i)
            
            # Reset to idle
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.idle_cmd)
            self.board.receiveDataPacket()
            time.sleep(1)
        
        # Print the detected mapping
        logger.info("Detected command mapping:")
        for i in range(4):
            channel = detected_mapping.get(i, "unknown")
            logger.info("Index %d -> %s", i, channel)
        
        # Determine if the mapping matches the expected order
        expected_order = ['roll', 'pitch', 'yaw', 'throttle']
        detected_order = [detected_mapping.get(i, "unknown") for i in range(4)]
        
        if detected_order == expected_order:
            logger.info("Command order matches expected order: %s", expected_order)
        else:
            logger.warning("Command order differs from expected order!")
            logger.warning("Expected: %s", expected_order)
            logger.warning("Detected: %s", detected_order)
        
        # Log to file
        self._log_to_file("command_order", {
            "detected_mapping": detected_mapping,
            "expected_order": expected_order,
            "detected_order": detected_order
        })
        
        # Publish to debug topic
        if self.debug_pub:
            debug_data = {
                "type": "command_order",
                "detected_mapping": detected_mapping,
                "expected_order": expected_order,
                "detected_order": detected_order,
                "timestamp": time.time()
            }
            self.debug_pub.publish(json.dumps(debug_data))
        
        return detected_mapping
    
    def initialize_ros(self):
        """Initialize ROS publishers for debugging"""
        try:
            rospy.init_node('flight_controller_debugger', anonymous=True)
            
            self.imu_pub = rospy.Publisher('/pidrone/imu_debug', Imu, queue_size=1, tcp_nodelay=False)
            self.mode_pub = rospy.Publisher('/pidrone/mode_debug', Mode, queue_size=1, tcp_nodelay=False)
            self.rc_pub = rospy.Publisher('/pidrone/rc_debug', RC, queue_size=1, tcp_nodelay=False)
            self.debug_pub = rospy.Publisher('/pidrone/debug_data', String, queue_size=10)
            self.log_pub = rospy.Publisher('/pidrone/debug_log', String, queue_size=100)
            
            logger.info("ROS publishers initialized")
            return True
        except Exception as e:
            logger.error("Failed to initialize ROS: %s", e)
            return False
    
    def save_log_to_file(self, filename=None):
        """Save current debug data to a file"""
        if not filename:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"fc_debug_export_{timestamp}.json"
        
        log_data = {
            "timestamp": time.time(),
            "command_history": [(cmd, desc, ts) for cmd, desc, ts in self.command_history],
            "last_attitude": self.last_attitude,
            "last_raw_imu": self.last_raw_imu,
            "last_rc": self.last_rc,
            "mode": self.mode
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(log_data, f, indent=2)
            
            logger.info(f"Log saved to {filename}")
            
            # Publish filename to log topic
            if self.log_pub:
                self.log_pub.publish(f"Log saved to {filename}")
            
            return True
        except Exception as e:
            logger.error(f"Failed to save log: {e}")
            return False
    
    def close(self):
        """Clean up resources"""
        self.stop_monitoring()
        if self.board:
            self.board.close()
        logger.info("Flight controller debugger closed")

def main():
    parser = argparse.ArgumentParser(description='Debug flight controller and MSP protocol')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port for the flight controller')
    parser.add_argument('--action', choices=['monitor', 'verify_order', 'test_channels', 'ros_bridge'], 
                       default='monitor', help='Action to perform')
    parser.add_argument('--duration', type=int, default=30, help='Duration for monitoring (seconds)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    parser.add_argument('--log-file', action='store_true', help='Log data to file')
    parser.add_argument('--export', type=str, help='Export logs to specified file')
    
    args = parser.parse_args()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Create the debugger
    debugger = FlightControllerDebugger(args.port, log_to_file=args.log_file)
    
    try:
        # Initialize ROS for all actions to enable topic publishing
        debugger.initialize_ros()
        
        if args.action == 'monitor':
            # Start monitoring
            debugger.start_monitoring()
            logger.info("Monitoring for %d seconds...", args.duration)
            time.sleep(args.duration)
        
        elif args.action == 'verify_order':
            # Verify command order
            debugger.verify_command_order()
        
        elif args.action == 'test_channels':
            # Test all channels
            debugger.test_all_channels()
        
        elif args.action == 'ros_bridge':
            # Start monitoring
            debugger.start_monitoring()
            logger.info("ROS bridge active. Press Ctrl+C to stop.")
            
            # Keep the main thread alive
            while True:
                time.sleep(1)
        
        # Export logs if requested
        if args.export:
            debugger.save_log_to_file(args.export)
    
    except KeyboardInterrupt:
        logger.info("Operation interrupted by user")
    except Exception as e:
        logger.error("Error: %s", e)
    finally:
        debugger.close()

if __name__ == "__main__":
    main() 