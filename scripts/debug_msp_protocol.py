#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import argparse
import struct
import logging
import json
import os
from h2rMultiWii import MultiWii

# Add ROS support if available
try:
    import rospy
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS not available, running without ROS support")

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('msp_debug')

def hexdump(data):
    """Format data as hex dump for debugging"""
    return ' '.join(['{:02x}'.format(ord(c)) for c in data])

def analyze_msp_packet(packet):
    """Analyze an MSP packet and return its components"""
    if len(packet) < 6:  # Minimum packet length
        return "Invalid packet: too short"
    
    # Extract header
    if packet[0:3] != '$M<':
        return "Invalid packet: wrong header"
    
    # Extract data length and command code
    data_length = ord(packet[3])
    command_code = ord(packet[4])
    
    # Extract data and checksum
    data = packet[5:5+data_length]
    checksum_byte = ord(packet[5+data_length]) if 5+data_length < len(packet) else None
    
    # Calculate checksum
    calculated_checksum = data_length ^ command_code
    for byte in data:
        calculated_checksum ^= ord(byte)
    
    checksum_valid = (checksum_byte == calculated_checksum)
    
    return {
        'header': '$M<',
        'data_length': data_length,
        'command_code': command_code,
        'data': hexdump(data),
        'checksum': checksum_byte,
        'calculated_checksum': calculated_checksum,
        'checksum_valid': checksum_valid
    }

class MSPProtocolDebugger:
    def __init__(self, port='/dev/ttyACM0', log_to_file=False):
        self.port = port
        self.board = None
        self.log_to_file = log_to_file
        self.log_file = None
        self.data_log = []
        
        # ROS publishers
        self.debug_pub = None
        self.log_pub = None
        self.packet_pub = None
        
        # Connect to the board
        self.connect()
        
        # Initialize log file if needed
        if self.log_to_file:
            self._init_log_file()
    
    def connect(self):
        """Connect to the flight controller board"""
        try:
            logger.info("Connecting to flight controller on %s...", self.port)
            self.board = MultiWii(self.port)
            logger.info("Connected!")
            return True
        except Exception as e:
            logger.error("Failed to connect: %s", e)
            return False
    
    def _init_log_file(self):
        """Initialize log file with timestamp in name"""
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_file = os.path.join(log_dir, "msp_protocol_{}.log".format(timestamp))
        
        with open(self.log_file, 'w') as f:
            f.write("# MSP Protocol Debug Log\n")
            f.write("# Started: {}\n".format(time.strftime('%Y-%m-%d %H:%M:%S')))
            f.write("# Format: timestamp,type,data\n")
        
        logger.info("Logging to file: {}".format(self.log_file))
    
    def _log_to_file(self, log_type, data):
        """Log data to file"""
        if not self.log_to_file or not self.log_file:
            return
        
        try:
            with open(self.log_file, 'a') as f:
                timestamp = time.time()
                json_data = json.dumps(data)
                f.write("{},{},{}\n".format(timestamp, log_type, json_data))
        except Exception as e:
            logger.error("Failed to write to log file: {}".format(e))
    
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
            logger.error("Failed to publish to ROS: {}".format(e))
    
    def _publish_packet_to_ros(self, direction, packet_data):
        """Publish packet data to ROS topic if available"""
        if not ROS_AVAILABLE or not self.packet_pub:
            return
        
        try:
            msg_data = {
                "direction": direction,
                "data": packet_data,
                "timestamp": time.time()
            }
            self.packet_pub.publish(json.dumps(msg_data))
        except Exception as e:
            logger.error("Failed to publish packet to ROS: {}".format(e))

    def monitor_raw_communication(self, duration=10):
        """Monitor raw communication with the flight controller"""
        logger.info("\nMonitoring raw communication for %s seconds...", duration)
        
        # Patch the MultiWii class to intercept communication
        original_write = self.board.ser.write
        original_read = self.board.ser.read
        
        # Store captured data
        captured_writes = []
        captured_reads = []
        
        def intercepted_write(data):
            captured_writes.append(data)
            packet_info = {"raw": hexdump(data)}
            analysis = analyze_msp_packet(data)
            if isinstance(analysis, dict):
                packet_info.update(analysis)
            
            logger.info("WRITE: %s", hexdump(data))
            
            # Log to file
            self._log_to_file("write_packet", packet_info)
            
            # Publish to ROS
            self._publish_to_ros("write_packet", packet_info)
            self._publish_packet_to_ros("write", packet_info)
            
            # Add to data log
            self.data_log.append({"type": "write_packet", "data": packet_info, "timestamp": time.time()})
            
            return original_write(data)
        
        def intercepted_read(size=1):
            data = original_read(size)
            if data:
                captured_reads.append(data)
                packet_info = {"raw": hexdump(data)}
                
                logger.info("READ: %s", hexdump(data))
                
                # Log to file
                self._log_to_file("read_packet", packet_info)
                
                # Publish to ROS
                self._publish_to_ros("read_packet", packet_info)
                self._publish_packet_to_ros("read", packet_info)
                
                # Add to data log
                self.data_log.append({"type": "read_packet", "data": packet_info, "timestamp": time.time()})
            
            return data
        
        # Apply patches
        self.board.ser.write = intercepted_write
        self.board.ser.read = intercepted_read
        
        try:
            # Send some commands to observe the communication
            start_time = time.time()
            while time.time() - start_time < duration:
                # Request attitude data
                self.board.getData(MultiWii.ATTITUDE)
                time.sleep(0.5)
                
                # Request RC data
                self.board.getData(MultiWii.RC)
                time.sleep(0.5)
                
                # Request raw IMU data
                self.board.getData(MultiWii.RAW_IMU)
                time.sleep(0.5)
        finally:
            # Restore original methods
            self.board.ser.write = original_write
            self.board.ser.read = original_read
        
        # Analyze captured data
        logger.info("\nAnalysis of captured communication:")
        logger.info("==================================")
        
        logger.info("\nWRITE packets:")
        for i, packet in enumerate(captured_writes):
            logger.info("\nPacket #%d: %s", i+1, hexdump(packet))
            try:
                analysis = analyze_msp_packet(packet)
                if isinstance(analysis, dict):
                    logger.info("  Header: %s", analysis['header'])
                    logger.info("  Data Length: %s", analysis['data_length'])
                    logger.info("  Command Code: %s (%s)", analysis['command_code'], get_command_name(analysis['command_code']))
                    logger.info("  Data: %s", analysis['data'])
                    logger.info("  Checksum: %s (calculated: %s, valid: %s)",
                        analysis['checksum'], 
                        analysis['calculated_checksum'],
                        "Yes" if analysis['checksum_valid'] else "No"
                    )
                else:
                    logger.info("  %s", analysis)
            except Exception as e:
                logger.error("  Error analyzing packet: %s", e)
        
        logger.info("\nREAD packets:")
        # Read packets are more complex to analyze as they may be fragmented
        # Just show the raw data for now
        for i, packet in enumerate(captured_reads):
            logger.info("\nPacket #%d: %s", i+1, hexdump(packet))
        
        # Save analysis to file and ROS
        analysis_data = {
            "write_packets": len(captured_writes),
            "read_packets": len(captured_reads),
            "duration": duration
        }
        self._log_to_file("communication_analysis", analysis_data)
        self._publish_to_ros("communication_analysis", analysis_data)
        self.data_log.append({"type": "communication_analysis", "data": analysis_data, "timestamp": time.time()})
        
        return captured_writes, captured_reads

    def test_command_order(self):
        """Test different command orders to determine the correct mapping"""
        logger.info("\nTesting command order...")
        
        # Standard order: [roll, pitch, yaw, throttle, aux1, ...]
        test_commands = [
            {
                'name': "Roll Right",
                'command': [1700, 1500, 1500, 1500, 1900, 1000, 1000, 1000]
            },
            {
                'name': "Pitch Forward",
                'command': [1500, 1700, 1500, 1500, 1900, 1000, 1000, 1000]
            },
            {
                'name': "Yaw Right",
                'command': [1500, 1500, 1700, 1500, 1900, 1000, 1000, 1000]
            },
            {
                'name': "Increase Throttle",
                'command': [1500, 1500, 1500, 1700, 1900, 1000, 1000, 1000]
            }
        ]
        
        results = []
        
        for test in test_commands:
            logger.info("\n=================================")
            logger.info("Testing: %s", test['name'])
            logger.info("=================================")
            
            # Send the command
            logger.info("Command: %s", test['command'])
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, test['command'])
            self.board.receiveDataPacket()
            
            # Log command to file and ROS
            cmd_data = {
                "name": test['name'],
                "command": test['command']
            }
            self._log_to_file("test_command", cmd_data)
            self._publish_to_ros("test_command", cmd_data)
            self.data_log.append({"type": "test_command", "data": cmd_data, "timestamp": time.time()})
            
            # Wait a moment
            time.sleep(1)
            
            # Read back RC values
            rc_data = self.board.getData(MultiWii.RC)
            rc_values = {}
            if rc_data:
                logger.info("\nRC Channels:")
                for key, value in rc_data.items():
                    if key not in ['elapsed', 'timestamp', 'cmd']:
                        logger.info("  %s: %s", key, value)
                        rc_values[key] = value
            
            # Read attitude
            attitude = self.board.getData(MultiWii.ATTITUDE)
            att_values = {}
            if attitude:
                logger.info("\nAttitude:")
                logger.info("  Roll:  %.2f degrees", attitude['angx'])
                logger.info("  Pitch: %.2f degrees", attitude['angy'])
                logger.info("  Yaw:   %.2f degrees", attitude['heading'])
                att_values = {
                    'roll': attitude['angx'],
                    'pitch': attitude['angy'],
                    'yaw': attitude['heading']
                }
            
            # Store results
            result = {
                'test_name': test['name'],
                'command': test['command'],
                'rc_values': rc_values,
                'attitude': att_values
            }
            results.append(result)
            
            # Log result to file and ROS
            self._log_to_file("test_result", result)
            self._publish_to_ros("test_result", result)
            self.data_log.append({"type": "test_result", "data": result, "timestamp": time.time()})
            
            # Return to center before next test
            center_command = [1500, 1500, 1500, 1500, 1900, 1000, 1000, 1000]
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, center_command)
            self.board.receiveDataPacket()
            time.sleep(2)  # Give it time to center
        
        # Analyze results to determine command order
        logger.info("\nAnalysis of command order tests:")
        logger.info("===============================")
        
        # This is a simplified analysis that could be improved
        detected_mapping = {}
        for i, result in enumerate(results):
            logger.info("\nTest: %s", result['test_name'])
            logger.info("Command sent with high value at index %d", i)
            
            # Look for the channel with the highest value
            max_channel = None
            max_value = 0
            for channel, value in result['rc_values'].items():
                if channel in ['roll', 'pitch', 'yaw', 'throttle'] and value > max_value:
                    max_channel = channel
                    max_value = value
            
            if max_channel:
                logger.info("Detected highest RC value in channel: %s", max_channel)
                detected_mapping[i] = max_channel
            else:
                logger.info("Could not determine affected channel")
        
        # Log analysis to file and ROS
        analysis = {
            'detected_mapping': detected_mapping,
            'results': results
        }
        self._log_to_file("command_order_analysis", analysis)
        self._publish_to_ros("command_order_analysis", analysis)
        self.data_log.append({"type": "command_order_analysis", "data": analysis, "timestamp": time.time()})
        
        return results, detected_mapping
    
    def initialize_ros(self):
        """Initialize ROS publishers"""
        if not ROS_AVAILABLE:
            return False
        
        try:
            rospy.init_node('msp_protocol_debugger', anonymous=True)
            
            self.debug_pub = rospy.Publisher('/pidrone/msp_protocol_debug', String, queue_size=10)
            self.log_pub = rospy.Publisher('/pidrone/msp_protocol_log', String, queue_size=10)
            self.packet_pub = rospy.Publisher('/pidrone/msp_packets', String, queue_size=100)
            
            logger.info("ROS publishers initialized")
            return True
        except Exception as e:
            logger.error("Failed to initialize ROS: %s", e)
            return False
    
    def save_log_to_file(self, filename=None):
        """Save the entire log to a JSON file"""
        try:
            if not filename:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = "msp_protocol_export_{}.json".format(timestamp)
            
            with open(filename, 'w') as f:
                json.dump(self.data_log, f, indent=2)
            
            logger.info("Log saved to {}".format(filename))
            
            # Publish to ROS if available
            if ROS_AVAILABLE and self.log_pub:
                self.log_pub.publish("Log saved to {}".format(filename))
            
            return filename
        except Exception as e:
            logger.error("Failed to save log: {}".format(e))
            return None
    
    def close(self):
        """Clean up resources"""
        if self.board:
            logger.info("Closing connection...")
            self.board.close()
            logger.info("Connection closed")

def get_command_name(code):
    """Get the name of an MSP command code"""
    commands = {
        100: "IDENT",
        101: "STATUS",
        102: "RAW_IMU",
        103: "SERVO",
        104: "MOTOR",
        105: "RC",
        106: "RAW_GPS",
        107: "COMP_GPS",
        108: "ATTITUDE",
        109: "ALTITUDE",
        110: "ANALOG",
        111: "RC_TUNING",
        112: "PID",
        200: "SET_RAW_RC",
        # Add more as needed
    }
    return commands.get(code, "UNKNOWN")

def main():
    parser = argparse.ArgumentParser(description='Debug MSP protocol')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port for the flight controller')
    parser.add_argument('--action', choices=['monitor', 'test_order', 'all'], default='all', help='Action to perform')
    parser.add_argument('--duration', type=int, default=10, help='Duration for monitoring (seconds)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    parser.add_argument('--log-file', action='store_true', help='Log data to file')
    parser.add_argument('--export', type=str, help='Export logs to specified file')
    parser.add_argument('--ros', action='store_true', help='Enable ROS publishing')
    
    args = parser.parse_args()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Create debugger
    debugger = MSPProtocolDebugger(args.port, log_to_file=args.log_file)
    
    try:
        # Initialize ROS if requested
        if args.ros and ROS_AVAILABLE:
            debugger.initialize_ros()
        
        # Perform the requested action
        if args.action == 'monitor' or args.action == 'all':
            debugger.monitor_raw_communication(args.duration)
        
        if args.action == 'test_order' or args.action == 'all':
            debugger.test_command_order()
        
        # Export logs if requested
        if args.export:
            debugger.save_log_to_file(args.export)
        
    except KeyboardInterrupt:
        logger.info("\nOperation interrupted by user.")
    except Exception as e:
        logger.error("Error: %s", e)
    finally:
        debugger.close()

if __name__ == "__main__":
    main() 