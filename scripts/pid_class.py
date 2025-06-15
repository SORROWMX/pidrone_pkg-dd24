#!/usr/bin/env python

import rospy
import time
import numpy as np
from pidrone_pkg.msg import RC


#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, midpoint=1500, control_range=None,
                 i_range=None, d_range=None, d_filter_size=10, init_i=0):
        """
        Initialize the PID controller.

        Args:
            kp: proportional gain
            ki: integral gain
            kd: derivative gain
            midpoint: midpoint of the control range
            control_range: tuple of (min, max) for the control output
            i_range: tuple of (min, max) for the integral term
            d_range: tuple of (min, max) for the derivative term
            d_filter_size: size of the filter for the derivative term
            init_i: initial value for the integral term
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.midpoint = midpoint
        self.control_range = control_range
        self.i_range = i_range
        self.d_range = d_range
        self.d_filter_size = d_filter_size
        self.init_i = init_i

        self._i = init_i
        self._prev_err = None
        self._prev_time = None
        self._d_filter = []

    def step(self, err, dt=None):
        """
        Compute the control output for a single time step.

        Args:
            err: current error
            dt: time step (if None, will use time since last call)

        Returns:
            control output
        """
        # Calculate dt if not provided
        if dt is None:
            curr_time = time.time()
            if self._prev_time is None:
                dt = 0
            else:
                dt = curr_time - self._prev_time
            self._prev_time = curr_time

        # Proportional term
        p = self.kp * err

        # Integral term
        self._i += self.ki * err * dt
        if self.i_range is not None:
            self._i = max(min(self._i, self.i_range[1]), self.i_range[0])

        # Derivative term (with filtering)
        if self._prev_err is None:
            d = 0
        else:
            d_raw = (err - self._prev_err) / dt if dt > 0 else 0
            
            # Apply filtering to derivative term
            self._d_filter.append(d_raw)
            if len(self._d_filter) > self.d_filter_size:
                self._d_filter.pop(0)
            
            # Use median filter to remove spikes
            d = np.median(self._d_filter) * self.kd
            
            if self.d_range is not None:
                d = max(min(d, self.d_range[1]), self.d_range[0])

        self._prev_err = err

        # Calculate control output
        control = self.midpoint + p + self._i + d

        # Limit control output
        if self.control_range is not None:
            control = max(min(control, self.control_range[1]), self.control_range[0])

        return int(control)

    def reset(self):
        """Reset the PID controller"""
        self._i = self.init_i
        self._prev_err = None
        self._prev_time = None
        self._d_filter = []


class PID:
    """A class that implements a generic PID controller for the drone."""
    def __init__(self):
        """Initialize the PID controller with default values"""
        # Improved settings for height control with better damping
        self.throttle = PIDaxis(0.4,  # Reduced P component for smoother response
                               0.2,   # Reduced I component to prevent overshoot
                               0.6,   # Increased D component for better damping
                               i_range=(-150, 150), control_range=(1200, 1450),
                               d_range=(-50, 50), midpoint=1350, d_filter_size=15)

        # Roll PID controllers (low and high rates)
        self.roll_low = PIDaxis(0.5, 0.02, 0.2,
                               i_range=(-150, 150), control_range=(1400, 1600),
                               d_range=(-50, 50), midpoint=1500, init_i=0)
        self.roll_high = PIDaxis(0.5, 0.02, 0.2,
                                i_range=(-150, 150), control_range=(1400, 1600),
                                d_range=(-50, 50), midpoint=1500, init_i=0)

        # Pitch PID controllers (low and high rates)
        self.pitch_low = PIDaxis(0.5, 0.02, 0.2,
                                i_range=(-150, 150), control_range=(1400, 1600),
                                d_range=(-50, 50), midpoint=1500, init_i=0)
        self.pitch_high = PIDaxis(0.5, 0.02, 0.2,
                                 i_range=(-150, 150), control_range=(1400, 1600),
                                 d_range=(-50, 50), midpoint=1500, init_i=0)

        # Yaw PID controller
        self.yaw = PIDaxis(0.5, 0.02, 0.2,
                          i_range=(-150, 150), control_range=(1400, 1600),
                          d_range=(-50, 50), midpoint=1500)

        # Adaptive control parameters
        self.height_threshold = 0.3  # Threshold for switching between low and high rates
        self.velocity_threshold = 0.2  # Threshold for switching between low and high rates
        
        # Exponential filter for throttle
        self.throttle_filter_alpha = 0.7  # Filter coefficient (0-1)
        self.last_throttle_cmd = 1350  # Initial throttle value

    def step(self, pid_error, desired_yaw_velocity=0):
        """
        Compute the control outputs for a single time step.

        Args:
            pid_error: current error in [x, y, z] (velocity errors for x,y and position error for z)
            desired_yaw_velocity: desired yaw velocity

        Returns:
            [roll, pitch, throttle, yaw] control outputs - order matches AETR map in INAV
        """
        # Extract errors
        x_error = pid_error.x
        y_error = pid_error.y
        z_error = pid_error.z

        # Determine which rate to use based on error magnitude
        if abs(x_error) > self.velocity_threshold:
            roll_cmd = self.roll_high.step(x_error)
        else:
            roll_cmd = self.roll_low.step(x_error)

        if abs(y_error) > self.velocity_threshold:
            pitch_cmd = self.pitch_high.step(y_error)
        else:
            pitch_cmd = self.pitch_low.step(y_error)

        # Calculate yaw command
        yaw_cmd = self.yaw.step(desired_yaw_velocity)

        # Calculate throttle command with improved height control
        raw_throttle_cmd = self.throttle.step(z_error)
        
        # Apply exponential filter to throttle for smoother response
        filtered_throttle_cmd = (self.throttle_filter_alpha * raw_throttle_cmd + 
                                (1 - self.throttle_filter_alpha) * self.last_throttle_cmd)
        
        # Update last throttle command
        self.last_throttle_cmd = filtered_throttle_cmd
        
        # Return control commands in order [roll, pitch, throttle, yaw] to match AETR map
        return [roll_cmd, pitch_cmd, int(filtered_throttle_cmd), yaw_cmd]

    def reset(self):
        """Reset all PID controllers"""
        self.throttle.reset()
        self.roll_low.reset()
        self.roll_high.reset()
        self.pitch_low.reset()
        self.pitch_high.reset()
        self.yaw.reset()
        self.last_throttle_cmd = 1350  # Reset throttle filter
