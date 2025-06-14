#!/usr/bin/env python

import rospy
import numpy as np
import time
import tf
import math
from threading import Lock, Thread

from h2rMultiWii import MultiWii
from serial import SerialException
from std_msgs.msg import Empty, Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from pidrone_pkg.msg import Mode, RC, State

import command_values as cmds
from msp_offboard_msgs import NavigateRequest, NavigateResponse, SetVelocityRequest, SetVelocityResponse, SetPositionRequest, SetPositionResponse
from msp_offboard_msgs import navigate_service_handler, set_velocity_service_handler, set_position_service_handler

class MSPOffboard:
    """
    MSP-analog of simple_offboard for controlling the drone via MSP protocol.
    Provides a high-level interface for flight control.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('msp_offboard')
        
        # Connect to flight controller
        try:
            self.board = MultiWii('/dev/ttyACM0')
        except SerialException:
            try:
                self.board = MultiWii('/dev/ttyACM1')
            except SerialException:
                rospy.logerr('Failed to connect to flight controller!')
                raise
        
        # Flight parameters
        self.default_speed = 0.5  # m/s
        self.default_takeoff_height = 0.3  # m
        self.height_tolerance = 0.05  # m
        self.position_tolerance = 0.1  # m
        self.stabilization_time = 2.0  # seconds
        
        # Current state
        self.armed = False
        self.flying = False
        self.current_height = 0.0
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.target_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.target_yaw = 0.0
        self.control_mode = 'position'  # 'position' or 'velocity'
        
        # Locks for thread safety
        self.state_lock = Lock()
        self.command_lock = Lock()
        
        # Commands for flight controller
        self.current_command = list(cmds.disarm_cmd)
        
        # Timers
        self.command_timer = None
        self.telemetry_timer = None
        
        # Topic subscriptions
        rospy.Subscriber('/pidrone/range', Range, self.range_callback)
        rospy.Subscriber('/pidrone/state', State, self.state_callback)
        rospy.Subscriber('/pidrone/picamera/twist', TwistStamped, self.velocity_callback)
        
        # Publishers
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.rc_pub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1)
        self.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/msp_offboard', Empty, queue_size=1)
        self.height_stable_pub = rospy.Publisher('/pidrone/height_stable', Bool, queue_size=1)
        
        # Services
        rospy.Service('/pidrone/msp_offboard/takeoff', Trigger, self.takeoff)
        rospy.Service('/pidrone/msp_offboard/land', Trigger, self.land)
        rospy.Service('/pidrone/msp_offboard/navigate', Trigger, navigate_service_handler(self.navigate))
        rospy.Service('/pidrone/msp_offboard/set_velocity', Trigger, set_velocity_service_handler(self.set_velocity))
        rospy.Service('/pidrone/msp_offboard/set_position', Trigger, set_position_service_handler(self.set_position))
        rospy.Service('/pidrone/msp_offboard/get_telemetry', Trigger, self.get_telemetry_service)
        
        # Start timers
        self.command_timer = rospy.Timer(rospy.Duration(0.05), self.command_timer_callback)
        self.telemetry_timer = rospy.Timer(rospy.Duration(0.1), self.telemetry_timer_callback)
        self.heartbeat_timer = rospy.Timer(rospy.Duration(1.0), self.heartbeat_timer_callback)
        
        rospy.loginfo("MSP Offboard initialized")
    
    def range_callback(self, msg):
        """Handler for height data"""
        with self.state_lock:
            self.current_height = msg.range
            self.current_position['z'] = msg.range
    
    def state_callback(self, msg):
        """Handler for drone state data"""
        with self.state_lock:
            # Update position
            self.current_position['x'] = msg.pose_with_covariance.pose.position.x
            self.current_position['y'] = msg.pose_with_covariance.pose.position.y
            
            # Update velocity
            self.current_velocity['x'] = msg.twist_with_covariance.twist.linear.x
            self.current_velocity['y'] = msg.twist_with_covariance.twist.linear.y
            self.current_velocity['z'] = msg.twist_with_covariance.twist.linear.z
    
    def velocity_callback(self, msg):
        """Handler for velocity data from optical flow"""
        with self.state_lock:
            # Update only if optical flow data is reliable
            if abs(msg.twist.linear.x) < 1.0 and abs(msg.twist.linear.y) < 1.0:
                self.current_velocity['x'] = msg.twist.linear.x
                self.current_velocity['y'] = msg.twist.linear.y
    
    def command_timer_callback(self, event):
        """Send commands to flight controller"""
        with self.command_lock:
            if self.flying:
                # Generate command based on target position/velocity
                self.update_command()
                
                # Send command
                rc_msg = RC()
                rc_msg.roll = self.current_command[0]
                rc_msg.pitch = self.current_command[1]
                rc_msg.yaw = self.current_command[2]
                rc_msg.throttle = self.current_command[3]
                self.rc_pub.publish(rc_msg)
    
    def telemetry_timer_callback(self, event):
        """Get telemetry from flight controller"""
        try:
            # Get orientation data
            self.board.getData(MultiWii.ATTITUDE)
            
            # Check height stability
            if self.flying:
                height_error = abs(self.target_position['z'] - self.current_height)
                is_stable = height_error < self.height_tolerance
                self.height_stable_pub.publish(Bool(is_stable))
        except:
            rospy.logwarn("Error getting telemetry")
    
    def heartbeat_timer_callback(self, event):
        """Send heartbeat signal"""
        self.heartbeat_pub.publish(Empty())
    
    def update_command(self):
        """Update command based on target position/velocity"""
        # Choose control mode
        if self.control_mode == 'position':
            self._update_position_control()
        else:  # velocity
            self._update_velocity_control()
    
    def _update_position_control(self):
        """Update command in position control mode"""
        # Calculate position errors
        pos_error_x = self.target_position['x'] - self.current_position['x']
        pos_error_y = self.target_position['y'] - self.current_position['y']
        pos_error_z = self.target_position['z'] - self.current_height
        
        # Coefficients for converting errors to commands
        kp_xy = 100  # Coefficient for horizontal movement
        kp_z = 200   # Coefficient for vertical movement
        
        # Limit maximum error for safety
        pos_error_x = max(min(pos_error_x, 0.5), -0.5)
        pos_error_y = max(min(pos_error_y, 0.5), -0.5)
        pos_error_z = max(min(pos_error_z, 0.2), -0.2)
        
        # Convert errors to RC commands (1000-2000)
        roll_cmd = 1500 + int(kp_xy * pos_error_x)
        pitch_cmd = 1500 + int(kp_xy * pos_error_y)
        throttle_cmd = 1500 + int(kp_z * pos_error_z)
        yaw_cmd = 1500  # Not using yaw control for now
        
        # Limit commands
        roll_cmd = max(min(roll_cmd, 1700), 1300)
        pitch_cmd = max(min(pitch_cmd, 1700), 1300)
        throttle_cmd = max(min(throttle_cmd, 1700), 1300)
        
        # Update command
        self.current_command = [roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd, 1900, 1000, 1000, 1000]
    
    def _update_velocity_control(self):
        """Update command in velocity control mode"""
        # Coefficients for converting velocities to commands
        kv_xy = 200  # Coefficient for horizontal velocity
        kv_z = 300   # Coefficient for vertical velocity
        
        # Convert velocities to RC commands (1000-2000)
        roll_cmd = 1500 + int(kv_xy * self.target_velocity['x'])
        pitch_cmd = 1500 + int(kv_xy * self.target_velocity['y'])
        throttle_cmd = 1500 + int(kv_z * self.target_velocity['z'])
        yaw_cmd = 1500 + int(self.target_yaw * 100)  # Coefficient for angular velocity
        
        # Limit commands
        roll_cmd = max(min(roll_cmd, 1700), 1300)
        pitch_cmd = max(min(pitch_cmd, 1700), 1300)
        throttle_cmd = max(min(throttle_cmd, 1700), 1300)
        yaw_cmd = max(min(yaw_cmd, 1700), 1300)
        
        # Update command
        self.current_command = [roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd, 1900, 1000, 1000, 1000]
    
    def arm(self):
        """Enable motors using aux1 channel"""
        # Set aux1 channel > 1500 to arm
        arm_cmd = list(cmds.idle_cmd)
        arm_cmd[4] = 1800  # aux1 > 1500 for arming
        
        # Send command to flight controller
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, arm_cmd)
        self.board.receiveDataPacket()
        
        # Also publish arm mode via ROS
        mode_msg = Mode()
        mode_msg.mode = "ARMED"
        self.mode_pub.publish(mode_msg)
        
        rospy.sleep(1.0)  # Wait for motors to arm
        
        with self.state_lock:
            self.armed = True
        
        return True
    
    def disarm(self):
        """Disable motors using aux1 channel"""
        # Set aux1 channel < 1500 to disarm
        disarm_cmd = list(cmds.idle_cmd)
        disarm_cmd[4] = 1000  # aux1 < 1500 for disarming
        
        # Send command to flight controller
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
        self.board.receiveDataPacket()
        
        # Also publish disarm mode via ROS
        mode_msg = Mode()
        mode_msg.mode = "DISARMED"
        self.mode_pub.publish(mode_msg)
        
        with self.state_lock:
            self.armed = False
            self.flying = False
        
        return True
    
    def takeoff(self, req):
        """Takeoff to specified height"""
        if self.flying:
            return TriggerResponse(success=False, message="Drone is already flying")
        
        # Enable motors
        if not self.armed:
            self.arm()
        
        # Switch to flight mode
        mode_msg = Mode()
        mode_msg.mode = "FLYING"
        self.mode_pub.publish(mode_msg)
        
        # Switch to position control
        self.position_control_pub.publish(Bool(True))
        
        # Set target height and control mode
        with self.state_lock:
            self.target_position['z'] = self.default_takeoff_height
            self.control_mode = 'position'
            self.flying = True
        
        # Start smooth takeoff in separate thread
        Thread(target=self._smooth_takeoff).start()
        
        return TriggerResponse(success=True, message="Takeoff started")
    
    def _smooth_takeoff(self):
        """Smooth takeoff with stabilization check at each step"""
        initial_height = 0.05
        target_height = self.default_takeoff_height
        steps = 10
        height_step = (target_height - initial_height) / steps
        
        # Start with minimum height
        with self.state_lock:
            self.target_position['z'] = initial_height
        
        rospy.loginfo("Starting takeoff. Target height: {0} m".format(target_height))
        
        # Gradually increase height
        for i in range(1, steps + 1):
            current_target = initial_height + height_step * i
            
            with self.state_lock:
                self.target_position['z'] = current_target
            
            rospy.loginfo("Takeoff step {0}/{1}: height {2:.2f} m".format(i, steps, current_target))
            
            # Wait for height to be reached
            stable_count = 0
            timeout_count = 0
            max_timeout = 50  # Max 5 seconds for stabilization
            
            # Adaptive height stabilization logic
            while stable_count < 5 and timeout_count < max_timeout:
                with self.state_lock:
                    current_height = self.current_height
                    height_error = self.target_position['z'] - current_height
                
                # Check if drone is too high
                if current_height > 0.6:  # If drone is too high
                    rospy.logwarn("Height too high ({:.2f}m), adjusting target down".format(current_height))
                    # Decrease target height instead of disarming
                    with self.state_lock:
                        self.target_position['z'] = current_height - 0.1  # Reduce by 10 cm
                    # Reset timeout
                    timeout_count = 0
                    # Reset stability counter
                    stable_count = 0
                
                # Check if target height is reached
                if abs(height_error) < self.height_tolerance:
                    stable_count += 1
                else:
                    stable_count = 0
                    
                    # Adaptive correction when having difficulty reaching height
                    if timeout_count > 30:  # If we've been trying for too long
                        if height_error > 0:  # If we can't reach higher altitude
                            # Decrease target height
                            adjusted_target = current_height + height_error * 0.5
                            rospy.logwarn("Difficulty reaching height {:.2f}m, adjusting to {:.2f}m".format(
                                self.target_position['z'], adjusted_target))
                            with self.state_lock:
                                self.target_position['z'] = adjusted_target
                        else:  # If we can't descend to lower altitude
                            # Increase target height
                            adjusted_target = current_height + height_error * 0.5
                            rospy.logwarn("Difficulty descending to {:.2f}m, adjusting to {:.2f}m".format(
                                self.target_position['z'], adjusted_target))
                            with self.state_lock:
                                self.target_position['z'] = adjusted_target
                        
                        # Reset timeout after adjustment
                        timeout_count = 0
                
                timeout_count += 1
                rospy.sleep(0.1)
            
            if timeout_count >= max_timeout:
                rospy.logwarn("Timeout reaching height {0:.2f} m, continuing with current height {1:.2f} m".format(
                    current_target, self.current_height))
        
        # Final stabilization at reached height
        with self.state_lock:
            actual_height = self.current_height
            # Set actual reached height as target
            self.target_position['z'] = actual_height
        
        rospy.loginfo("Takeoff complete. Stabilized at height: {0:.2f} m".format(actual_height))
    
    def land(self, req):
        """Land the drone"""
        if not self.flying:
            return TriggerResponse(success=False, message="Drone is not flying")
        
        # Start smooth landing in separate thread
        Thread(target=self._smooth_landing).start()
        
        return TriggerResponse(success=True, message="Landing started")
    
    def _smooth_landing(self):
        """Smooth landing"""
        initial_height = self.current_height
        target_height = 0.05  # Minimum height before disarming
        steps = 10
        height_step = (initial_height - target_height) / steps
        
        rospy.loginfo("Starting landing from height {0:.2f} m".format(initial_height))
        
        # Gradually decrease height
        for i in range(1, steps + 1):
            current_target = initial_height - height_step * i
            
            with self.state_lock:
                self.target_position['z'] = current_target
            
            rospy.loginfo("Landing step {0}/{1}: height {2:.2f} m".format(i, steps, current_target))
            
            # Wait for height to be reached
            stable_count = 0
            timeout_count = 0
            while stable_count < 3 and timeout_count < 30:  # Max 3 seconds for stabilization
                with self.state_lock:
                    height_error = abs(self.target_position['z'] - self.current_height)
                
                if height_error < self.height_tolerance:
                    stable_count += 1
                else:
                    stable_count = 0
                
                timeout_count += 1
                rospy.sleep(0.1)
        
        # Disarm motors
        self.disarm()
        rospy.loginfo("Landing complete")
    
    def navigate(self, req):
        """Fly to specified point"""
        # Auto-arm if requested
        if req.auto_arm and not self.flying:
            rospy.loginfo("Auto-arm requested, performing takeoff first")
            # Send arm command directly via aux1 channel
            arm_cmd = list(self.current_command)
            arm_cmd[4] = 1800  # Set aux1 > 1500 to arm
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, arm_cmd)
            self.board.receiveDataPacket()
            rospy.sleep(1.0)  # Wait for arming
            
            # Now perform takeoff
            takeoff_result = self.takeoff(None)
            if not takeoff_result.success:
                response = NavigateResponse()
                response.success = False
                response.message = "Failed to takeoff: " + takeoff_result.message
                return response
            
            # Wait for stabilization after takeoff
            rospy.sleep(2.0)
        
        if not self.flying:
            response = NavigateResponse()
            response.success = False
            response.message = "Drone is not flying"
            return response
        
        # Check frame_id
        if req.frame_id == "body":
            # Relative coordinates (relative to current drone position)
            with self.state_lock:
                target_x = self.current_position['x'] + req.x
                target_y = self.current_position['y'] + req.y
                target_z = self.current_position['z'] + req.z
        else:
            # Absolute coordinates
            target_x = req.x
            target_y = req.y
            target_z = req.z
        
        # Limit height for safety
        target_z = max(min(target_z, 0.5), 0.05)
        
        # Save initial position for trajectory calculation
        with self.state_lock:
            start_x = self.current_position['x']
            start_y = self.current_position['y']
            start_z = self.current_position['z']
            
            # Save target position
            self.target_position['x'] = target_x
            self.target_position['y'] = target_y
            self.target_position['z'] = target_z
            self.control_mode = 'position'
        
        # Calculate movement speed
        speed = req.speed if req.speed > 0 else self.default_speed
        
        # Start flight in separate thread
        Thread(target=self._navigate_to_point, args=(start_x, start_y, start_z, target_x, target_y, target_z, speed)).start()
        
        response = NavigateResponse()
        response.success = True
        response.message = "Flight to point ({0:.2f}, {1:.2f}, {2:.2f}) started".format(target_x, target_y, target_z)
        return response
    
    def _navigate_to_point(self, start_x, start_y, start_z, target_x, target_y, target_z, speed):
        """Fly to specified point with specified speed using adaptive approach"""
        rospy.loginfo("Flying from ({:.2f}, {:.2f}, {:.2f}) to ({:.2f}, {:.2f}, {:.2f}) with speed {:.2f} m/s".format(
            start_x, start_y, start_z, target_x, target_y, target_z, speed))
        
        # Calculate total distance
        dx = target_x - start_x
        dy = target_y - start_y
        dz = target_z - start_z
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        if distance < 0.05:  # If distance is very small, set target directly
            rospy.loginfo("Target is very close, setting direct target")
            return
            
        # Calculate flight time based on speed
        flight_time = distance / speed
        
        # Determine number of steps (min 5, max 20)
        steps = min(max(int(flight_time * 10), 5), 20)
        
        rospy.loginfo("Flight will be divided into {} steps".format(steps))
        
        # Gradual movement to target
        for i in range(1, steps + 1):
            # Calculate intermediate point
            progress = i / float(steps)
            intermediate_x = start_x + dx * progress
            intermediate_y = start_y + dy * progress
            intermediate_z = start_z + dz * progress
            
            # Set intermediate goal
            with self.state_lock:
                self.target_position['x'] = intermediate_x
                self.target_position['y'] = intermediate_y
                self.target_position['z'] = intermediate_z
            
            rospy.loginfo("Navigation step {}/{}: position ({:.2f}, {:.2f}, {:.2f})".format(
                i, steps, intermediate_x, intermediate_y, intermediate_z))
            
            # Wait for intermediate point to be reached
            stable_count = 0
            timeout_count = 0
            max_timeout = 50  # Max 5 seconds for each step
            
            # Adaptive logic for reaching intermediate point
            while stable_count < 5 and timeout_count < max_timeout:
                with self.state_lock:
                    current_x = self.current_position['x']
                    current_y = self.current_position['y']
                    current_z = self.current_position['z']
                    
                    # Calculate position error
                    pos_error_x = self.target_position['x'] - current_x
                    pos_error_y = self.target_position['y'] - current_y
                    pos_error_z = self.target_position['z'] - current_z
                    
                    # Total position error
                    pos_error = (pos_error_x**2 + pos_error_y**2 + pos_error_z**2)**0.5
                
                # Check if point is reached
                if pos_error < self.position_tolerance:
                    stable_count += 1
                else:
                    stable_count = 0
                    
                    # Adaptive correction when having difficulty reaching point
                    if timeout_count > 30:
                        # Adjust target closer to current position
                        adjusted_x = current_x + pos_error_x * 0.5
                        adjusted_y = current_y + pos_error_y * 0.5
                        adjusted_z = current_z + pos_error_z * 0.5
                        
                        rospy.logwarn("Difficulty reaching position, adjusting intermediate target")
                        with self.state_lock:
                            self.target_position['x'] = adjusted_x
                            self.target_position['y'] = adjusted_y
                            self.target_position['z'] = adjusted_z
                        
                        timeout_count = 0  # Reset timeout after adjustment
                
                timeout_count += 1
                rospy.sleep(0.1)
            
            if timeout_count >= max_timeout:
                rospy.logwarn("Timeout reaching intermediate position, continuing to next step")
        
        # Final stabilization at target point
        rospy.loginfo("Reached target vicinity, stabilizing at final position")
        
        # Set final goal
        with self.state_lock:
            self.target_position['x'] = target_x
            self.target_position['y'] = target_y
            self.target_position['z'] = target_z
        
        # Wait for final stabilization
        stable_count = 0
        timeout_count = 0
        while stable_count < 10 and timeout_count < 100:  # Wait longer for final stabilization
            with self.state_lock:
                pos_error_x = self.target_position['x'] - self.current_position['x']
                pos_error_y = self.target_position['y'] - self.current_position['y']
                pos_error_z = self.target_position['z'] - self.current_position['z']
                pos_error = (pos_error_x**2 + pos_error_y**2 + pos_error_z**2)**0.5
            
            if pos_error < self.position_tolerance:
                stable_count += 1
            else:
                stable_count = 0
            
            timeout_count += 1
            rospy.sleep(0.1)
        
        with self.state_lock:
            final_x = self.current_position['x']
            final_y = self.current_position['y']
            final_z = self.current_position['z']
        
        if timeout_count >= 100:
            rospy.logwarn("Could not fully stabilize at target, final position: ({:.2f}, {:.2f}, {:.2f})".format(
                final_x, final_y, final_z))
        else:
            rospy.loginfo("Successfully reached target position: ({:.2f}, {:.2f}, {:.2f})".format(
                final_x, final_y, final_z))
    
    def set_velocity(self, req):
        """Set target velocity"""
        if not self.flying:
            response = SetVelocityResponse()
            response.success = False
            response.message = "Drone is not flying"
            return response
        
        # Limit velocity for safety
        vx = max(min(req.vx, 0.5), -0.5)
        vy = max(min(req.vy, 0.5), -0.5)
        vz = max(min(req.vz, 0.3), -0.3)
        yaw_rate = max(min(req.yaw_rate, 0.5), -0.5)
        
        # Set target velocity
        with self.state_lock:
            self.target_velocity['x'] = vx
            self.target_velocity['y'] = vy
            self.target_velocity['z'] = vz
            self.target_yaw = yaw_rate
            self.control_mode = 'velocity'
        
        response = SetVelocityResponse()
        response.success = True
        response.message = "Set velocity ({0:.2f}, {1:.2f}, {2:.2f})".format(vx, vy, vz)
        return response
    
    def set_position(self, req):
        """Set target position"""
        if not self.flying:
            response = SetPositionResponse()
            response.success = False
            response.message = "Drone is not flying"
            return response
        
        # Check frame_id
        if req.frame_id == "body":
            # Relative coordinates (relative to current drone position)
            with self.state_lock:
                target_x = self.current_position['x'] + req.x
                target_y = self.current_position['y'] + req.y
                target_z = self.current_position['z'] + req.z
        else:
            # Absolute coordinates
            target_x = req.x
            target_y = req.y
            target_z = req.z
        
        # Limit height for safety
        target_z = max(min(target_z, 0.5), 0.05)
        
        # Set target position
        with self.state_lock:
            self.target_position['x'] = target_x
            self.target_position['y'] = target_y
            self.target_position['z'] = target_z
            self.control_mode = 'position'
        
        response = SetPositionResponse()
        response.success = True
        response.message = "Set position ({0:.2f}, {1:.2f}, {2:.2f})".format(target_x, target_y, target_z)
        return response
    
    def get_telemetry(self, req=None):
        """Get current drone state"""
        with self.state_lock:
            telemetry = {
                'x': self.current_position['x'],
                'y': self.current_position['y'],
                'z': self.current_position['z'],
                'vx': self.current_velocity['x'],
                'vy': self.current_velocity['y'],
                'vz': self.current_velocity['z'],
                'armed': self.armed,
                'flying': self.flying,
                'mode': self.control_mode
            }
            
            # Get orientation from flight controller
            try:
                self.board.getData(MultiWii.ATTITUDE)
                roll = np.deg2rad(self.board.attitude['angx'])
                pitch = -np.deg2rad(self.board.attitude['angy'])
                heading = np.deg2rad(self.board.attitude['heading'])
                
                # Convert angles to quaternion
                quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
                
                # Add orientation to telemetry
                telemetry['roll'] = roll
                telemetry['pitch'] = pitch
                telemetry['yaw'] = heading
                telemetry['quaternion_x'] = quaternion[0]
                telemetry['quaternion_y'] = quaternion[1]
                telemetry['quaternion_z'] = quaternion[2]
                telemetry['quaternion_w'] = quaternion[3]
            except:
                rospy.logwarn("Failed to get orientation data")
            
        return telemetry
    
    def get_telemetry_service(self, req):
        """Service for getting telemetry"""
        telemetry = self.get_telemetry()
        
        # Format response as string
        response = TriggerResponse()
        response.success = True
        
        # Create string with telemetry data
        telemetry_str = []
        for key, value in telemetry.items():
            if isinstance(value, float):
                telemetry_str.append("{}: {:.3f}".format(key, value))
            else:
                telemetry_str.append("{}: {}".format(key, value))
                
        response.message = "\n".join(telemetry_str)
        return response

if __name__ == '__main__':
    try:
        offboard = MSPOffboard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: {0}".format(e))
        import traceback
        traceback.print_exc() 