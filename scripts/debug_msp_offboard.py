#!/usr/bin/env python

import time
import sys
import os
import json
import argparse
import logging
import rospy
from threading import Thread
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Vector3, Pose, Twist
from std_msgs.msg import Empty, Bool, Float32, String
from pidrone_pkg.msg import Mode, RC, State
from sensor_msgs.msg import Range

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('msp_offboard_debug')

class MSPOffboardDebugger:
    """
    Debug tool for testing and analyzing the MSP Offboard system
    """
    def __init__(self):
        self.running = False
        self.altitude = 0.0
        self.state = None
        self.mode = "DISARMED"
        self.position_control = False
        self.data_log = []
        self.log_file = None
        
        # Initialize ROS node
        rospy.init_node('msp_offboard_debugger', anonymous=True)
        
        # Set up subscribers
        rospy.Subscriber('/pidrone/state', State, self.state_callback)
        rospy.Subscriber('/pidrone/range', Range, self.range_callback)
        rospy.Subscriber('/pidrone/mode', Mode, self.mode_callback)
        rospy.Subscriber('/pidrone/position_control', Bool, self.position_control_callback)
        
        # Set up service proxies
        logger.info("Waiting for MSP Offboard services...")
        try:
            rospy.wait_for_service('/pidrone/msp_offboard/takeoff', timeout=5)
            self.takeoff_service = rospy.ServiceProxy('/pidrone/msp_offboard/takeoff', Trigger)
            
            rospy.wait_for_service('/pidrone/msp_offboard/land', timeout=5)
            self.land_service = rospy.ServiceProxy('/pidrone/msp_offboard/land', Trigger)
            
            rospy.wait_for_service('/pidrone/msp_offboard/navigate', timeout=5)
            self.navigate_service = rospy.ServiceProxy('/pidrone/msp_offboard/navigate', Trigger)
            
            rospy.wait_for_service('/pidrone/msp_offboard/set_position', timeout=5)
            self.set_position_service = rospy.ServiceProxy('/pidrone/msp_offboard/set_position', Trigger)
            
            rospy.wait_for_service('/pidrone/msp_offboard/set_velocity', timeout=5)
            self.set_velocity_service = rospy.ServiceProxy('/pidrone/msp_offboard/set_velocity', Trigger)
            
            logger.info("All MSP Offboard services are available")
        except rospy.ROSException:
            logger.error("Failed to connect to MSP Offboard services. Make sure they are running.")
            sys.exit(1)
        
        # Set up publishers
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        self.twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/web_interface', Empty, queue_size=1)
        
        # Debug publishers
        self.debug_pub = rospy.Publisher('/pidrone/offboard_debug', String, queue_size=10)
        self.log_pub = rospy.Publisher('/pidrone/offboard_log', String, queue_size=10)
        
        # Start heartbeat thread
        self.start_heartbeat()
        
        # Log initialization
        self._log_event("init", {
            "message": "MSP Offboard Debugger initialized",
            "timestamp": time.time()
        })
        
        rospy.loginfo("MSP Offboard Debugger initialized")
    
    def _init_log_file(self, filename=None):
        """Initialize log file with timestamp in name"""
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        if not filename:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(log_dir, f"msp_offboard_{timestamp}.log")
        
        self.log_file = filename
        
        with open(self.log_file, 'w') as f:
            f.write("# MSP Offboard Debug Log\n")
            f.write(f"# Started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("# Format: timestamp,type,data\n")
        
        rospy.loginfo(f"Logging to file: {self.log_file}")
        return self.log_file
    
    def _log_to_file(self, log_type, data):
        """Log data to file if log file is initialized"""
        if not self.log_file:
            return
        
        try:
            with open(self.log_file, 'a') as f:
                timestamp = time.time()
                json_data = json.dumps(data)
                f.write(f"{timestamp},{log_type},{json_data}\n")
        except Exception as e:
            rospy.logerr(f"Failed to write to log file: {e}")
    
    def _log_event(self, event_type, data):
        """Log event data and publish to ROS topic"""
        # Add to internal log
        log_entry = {
            "type": event_type,
            "data": data,
            "timestamp": time.time()
        }
        self.data_log.append(log_entry)
        
        # Log to file if available
        self._log_to_file(event_type, data)
        
        # Publish to ROS topic
        try:
            self.debug_pub.publish(json.dumps(log_entry))
        except Exception as e:
            rospy.logerr(f"Failed to publish debug data: {e}")
    
    def start_heartbeat(self):
        """Start heartbeat thread to keep the flight controller active"""
        self.running = True
        self.heartbeat_thread = Thread(target=self._heartbeat_loop)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        logger.info("Heartbeat started")
    
    def _heartbeat_loop(self):
        """Send heartbeat messages at regular intervals"""
        empty_msg = Empty()
        rate = rospy.Rate(1)  # 1 Hz
        while self.running and not rospy.is_shutdown():
            try:
                self.heartbeat_pub.publish(empty_msg)
                rate.sleep()
            except Exception as e:
                logger.error("Error in heartbeat loop: %s", e)
    
    def state_callback(self, msg):
        """Handle state updates"""
        self.state = msg
        
        # Log position and velocity data periodically
        if hasattr(self, 'last_state_log_time'):
            if rospy.Time.now() - self.last_state_log_time > rospy.Duration(1.0):
                self._log_state_data()
                self.last_state_log_time = rospy.Time.now()
        else:
            self.last_state_log_time = rospy.Time.now()
    
    def _log_state_data(self):
        """Log current state data"""
        if self.state:
            pos = self.state.pose_with_covariance.pose.position
            vel = self.state.twist_with_covariance.twist.linear
            logger.debug("Position: x=%.3f y=%.3f z=%.3f", pos.x, pos.y, pos.z)
            logger.debug("Velocity: x=%.3f y=%.3f z=%.3f", vel.x, vel.y, vel.z)
    
    def range_callback(self, msg):
        """Handle altitude updates"""
        self.altitude = msg.range
    
    def mode_callback(self, msg):
        """Handle mode updates"""
        self.mode = msg.mode
        logger.debug("Mode: %s", self.mode)
    
    def position_control_callback(self, msg):
        """Handle position control mode updates"""
        self.position_control = msg.data
        logger.debug("Position control: %s", self.position_control)
    
    def arm(self):
        """Arm the drone"""
        logger.info("Arming drone...")
        mode_msg = Mode()
        mode_msg.mode = "ARMED"
        self.mode_pub.publish(mode_msg)
        
        # Wait for the mode to change
        start_time = rospy.Time.now()
        while self.mode != "ARMED" and rospy.Time.now() - start_time < rospy.Duration(3.0):
            rospy.sleep(0.1)
        
        if self.mode == "ARMED":
            logger.info("Drone armed successfully")
            return True
        else:
            logger.error("Failed to arm drone")
            return False
    
    def disarm(self):
        """Disarm the drone"""
        logger.info("Disarming drone...")
        mode_msg = Mode()
        mode_msg.mode = "DISARMED"
        self.mode_pub.publish(mode_msg)
        
        # Wait for the mode to change
        start_time = rospy.Time.now()
        while self.mode != "DISARMED" and rospy.Time.now() - start_time < rospy.Duration(3.0):
            rospy.sleep(0.1)
        
        if self.mode == "DISARMED":
            logger.info("Drone disarmed successfully")
            return True
        else:
            logger.error("Failed to disarm drone")
            return False
    
    def enable_position_control(self):
        """Enable position control mode"""
        logger.info("Enabling position control...")
        self.position_control_pub.publish(Bool(True))
        
        # Wait for the mode to change
        start_time = rospy.Time.now()
        while not self.position_control and rospy.Time.now() - start_time < rospy.Duration(3.0):
            rospy.sleep(0.1)
        
        if self.position_control:
            logger.info("Position control enabled")
            return True
        else:
            logger.error("Failed to enable position control")
            return False
    
    def disable_position_control(self):
        """Disable position control mode"""
        logger.info("Disabling position control...")
        self.position_control_pub.publish(Bool(False))
        
        # Wait for the mode to change
        start_time = rospy.Time.now()
        while self.position_control and rospy.Time.now() - start_time < rospy.Duration(3.0):
            rospy.sleep(0.1)
        
        if not self.position_control:
            logger.info("Position control disabled")
            return True
        else:
            logger.error("Failed to disable position control")
            return False
    
    def takeoff(self, height=0.3):
        """Call the takeoff service"""
        logger.info("Calling takeoff service (target height: %.2f m)...", height)
        try:
            response = self.takeoff_service()
            logger.info("Takeoff service response: %s - %s", response.success, response.message)
            
            # Monitor altitude
            if response.success:
                self._monitor_altitude(target=height, timeout=15, direction="up")
            
            return response.success
        except rospy.ServiceException as e:
            logger.error("Takeoff service call failed: %s", e)
            return False
    
    def land(self):
        """Call the land service"""
        logger.info("Calling land service...")
        try:
            response = self.land_service()
            logger.info("Land service response: %s - %s", response.success, response.message)
            
            # Monitor altitude
            if response.success:
                self._monitor_altitude(target=0.05, timeout=15, direction="down")
            
            return response.success
        except rospy.ServiceException as e:
            logger.error("Land service call failed: %s", e)
            return False
    
    def navigate_to_point(self, x=0.0, y=0.0, z=0.3):
        """Call the navigate service to move to a point"""
        logger.info("Navigating to point (%.2f, %.2f, %.2f)...", x, y, z)
        try:
            # Note: This is a simplified version. In reality, we would need to pass
            # the coordinates to the service, but we're using Trigger for simplicity.
            response = self.navigate_service()
            logger.info("Navigate service response: %s - %s", response.success, response.message)
            return response.success
        except rospy.ServiceException as e:
            logger.error("Navigate service call failed: %s", e)
            return False
    
    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """Call the set_velocity service"""
        logger.info("Setting velocity to (%.2f, %.2f, %.2f) m/s, yaw_rate: %.2f rad/s...", 
                   vx, vy, vz, yaw_rate)
        try:
            # Note: This is a simplified version. In reality, we would need to pass
            # the velocity values to the service, but we're using Trigger for simplicity.
            response = self.set_velocity_service()
            logger.info("Set velocity service response: %s - %s", response.success, response.message)
            return response.success
        except rospy.ServiceException as e:
            logger.error("Set velocity service call failed: %s", e)
            return False
    
    def _monitor_altitude(self, target, timeout, direction="up"):
        """Monitor altitude changes during takeoff or landing"""
        logger.info("Monitoring altitude (target: %.2f m, timeout: %d s)...", target, timeout)
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration(timeout):
            logger.info("Current altitude: %.3f m", self.altitude)
            
            if direction == "up" and self.altitude >= target * 0.9:
                logger.info("Reached target altitude")
                return True
            elif direction == "down" and self.altitude <= target * 1.1:
                logger.info("Reached target altitude")
                return True
            
            rate.sleep()
        
        logger.warning("Altitude monitoring timed out")
        return False
    
    def test_takeoff_land(self):
        """Test the takeoff and land sequence"""
        logger.info("Starting takeoff-land test...")
        
        # Arm the drone
        if not self.arm():
            return False
        
        # Wait a moment
        rospy.sleep(1.0)
        
        # Takeoff
        if not self.takeoff(height=0.3):
            self.disarm()
            return False
        
        # Hover for a few seconds
        logger.info("Hovering for 5 seconds...")
        rospy.sleep(5.0)
        
        # Land
        if not self.land():
            self.disarm()
            return False
        
        # Wait for landing to complete
        rospy.sleep(2.0)
        
        # Disarm
        self.disarm()
        
        logger.info("Takeoff-land test completed successfully")
        return True
    
    def test_square_flight(self, side_length=0.5, height=0.3):
        """Test flying in a square pattern"""
        logger.info("Starting square flight test (side length: %.2f m)...", side_length)
        
        # Arm the drone
        if not self.arm():
            return False
        
        # Takeoff
        if not self.takeoff(height=height):
            self.disarm()
            return False
        
        # Hover for a moment
        logger.info("Hovering for 2 seconds...")
        rospy.sleep(2.0)
        
        # Enable position control
        if not self.enable_position_control():
            self.land()
            self.disarm()
            return False
        
        # Fly in a square pattern
        waypoints = [
            (side_length, 0, height),  # Forward
            (side_length, side_length, height),  # Right
            (0, side_length, height),  # Backward
            (0, 0, height)   # Left (back to start)
        ]
        
        for i, (x, y, z) in enumerate(waypoints):
            logger.info("Flying to waypoint %d: (%.2f, %.2f, %.2f)...", i+1, x, y, z)
            if not self.navigate_to_point(x, y, z):
                logger.error("Failed to navigate to waypoint %d", i+1)
                break
            
            # Wait to reach the waypoint
            rospy.sleep(5.0)
        
        # Land
        if not self.land():
            self.disarm()
            return False
        
        # Wait for landing to complete
        rospy.sleep(2.0)
        
        # Disarm
        self.disarm()
        
        logger.info("Square flight test completed")
        return True
    
    def test_velocity_control(self, speed=0.2, duration=2.0):
        """Test velocity control in different directions"""
        logger.info("Starting velocity control test (speed: %.2f m/s)...", speed)
        
        # Arm the drone
        if not self.arm():
            return False
        
        # Takeoff
        if not self.takeoff(height=0.3):
            self.disarm()
            return False
        
        # Hover for a moment
        logger.info("Hovering for 2 seconds...")
        rospy.sleep(2.0)
        
        # Disable position control (use velocity control)
        if not self.disable_position_control():
            self.land()
            self.disarm()
            return False
        
        # Test forward velocity
        logger.info("Testing forward velocity...")
        if not self.set_velocity(vx=0, vy=speed, vz=0):
            self.land()
            self.disarm()
            return False
        rospy.sleep(duration)
        
        # Stop
        self.set_velocity(0, 0, 0)
        rospy.sleep(1.0)
        
        # Test backward velocity
        logger.info("Testing backward velocity...")
        if not self.set_velocity(vx=0, vy=-speed, vz=0):
            self.land()
            self.disarm()
            return False
        rospy.sleep(duration)
        
        # Stop
        self.set_velocity(0, 0, 0)
        rospy.sleep(1.0)
        
        # Test right velocity
        logger.info("Testing right velocity...")
        if not self.set_velocity(vx=speed, vy=0, vz=0):
            self.land()
            self.disarm()
            return False
        rospy.sleep(duration)
        
        # Stop
        self.set_velocity(0, 0, 0)
        rospy.sleep(1.0)
        
        # Test left velocity
        logger.info("Testing left velocity...")
        if not self.set_velocity(vx=-speed, vy=0, vz=0):
            self.land()
            self.disarm()
            return False
        rospy.sleep(duration)
        
        # Stop
        self.set_velocity(0, 0, 0)
        rospy.sleep(1.0)
        
        # Land
        if not self.land():
            self.disarm()
            return False
        
        # Wait for landing to complete
        rospy.sleep(2.0)
        
        # Disarm
        self.disarm()
        
        logger.info("Velocity control test completed")
        return True
    
    def save_log_to_file(self, filename=None):
        """Save current debug data to a file"""
        if not filename:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"msp_offboard_export_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.data_log, f, indent=2)
            
            rospy.loginfo(f"Log saved to {filename}")
            
            # Publish filename to log topic
            self.log_pub.publish(f"Log saved to {filename}")
            
            return True
        except Exception as e:
            rospy.logerr(f"Failed to save log: {e}")
            return False
    
    def close(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'heartbeat_thread'):
            self.heartbeat_thread.join(timeout=1.0)
        logger.info("MSP Offboard debugger closed")

def main():
    parser = argparse.ArgumentParser(description='Debug MSP Offboard system')
    parser.add_argument('--test', choices=['takeoff_land', 'square', 'velocity', 'monitor'], 
                       default='monitor', help='Test to run')
    parser.add_argument('--height', type=float, default=0.3, help='Flight height (m)')
    parser.add_argument('--side', type=float, default=0.5, help='Square side length (m)')
    parser.add_argument('--speed', type=float, default=0.2, help='Velocity for tests (m/s)')
    parser.add_argument('--duration', type=float, default=2.0, help='Duration for velocity commands (s)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    parser.add_argument('--log-file', action='store_true', help='Log data to file')
    parser.add_argument('--export', type=str, help='Export logs to specified file')
    
    args = parser.parse_args()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # Create the debugger
    debugger = MSPOffboardDebugger()
    
    # Initialize log file if requested
    if args.log_file:
        debugger._init_log_file()
    
    try:
        if args.test == 'takeoff_land':
            debugger.test_takeoff_land()
        elif args.test == 'square':
            debugger.test_square_flight(side_length=args.side, height=args.height)
        elif args.test == 'velocity':
            debugger.test_velocity_control(speed=args.speed, duration=args.duration)
        else:  # monitor
            logger.info("Monitoring MSP Offboard system. Press Ctrl+C to stop.")
            while not rospy.is_shutdown():
                rospy.sleep(1.0)
        
        # Export logs if requested
        if args.export:
            debugger.save_log_to_file(args.export)
        elif args.log_file:
            debugger.save_log_to_file()
    
    except KeyboardInterrupt:
        logger.info("Operation interrupted by user")
    except Exception as e:
        logger.error("Error: %s", e)
    finally:
        debugger.close()

if __name__ == "__main__":
    main() 