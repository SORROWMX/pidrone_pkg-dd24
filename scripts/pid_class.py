#!/usr/bin/env python

import rospy
import numpy as np
import math


#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, i_range=None, d_range=None, control_range=(1000, 2000), midpoint=1500, smoothing=True, d_filter_size=3):
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Config
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing
        self.d_filter_size = d_filter_size
        # Internal
        self.reset()

    def reset(self):
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0

    def step(self, err, time_elapsed):
        if self._old_err is None:
            # First time around prevent d term spike
            self._old_err = err

        # Find the p component
        self._p = err * self.kp

        # Find the i component
        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        # Find the d component
        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))
        self._old_err = err

        # Smooth over the last three d terms
        if self.smoothing:
            if self.d_filter_size == 3:
                self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            else:
                # More aggressive smoothing for larger filter size
                self._d = (self._d * 0.5 + self._dd * 0.3 + self._ddd * 0.2)
            self._ddd = self._dd
            self._dd = self._d

        # Calculate control output
        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]), self.control_range[1])

        return output


class EnhancedThrottlePID():
    """
    Enhanced throttle controller based on height_control_flight.py's calculate_throttle function.
    Provides more sophisticated height control with adaptive parameters.
    """
    def __init__(self, kp=90.0, ki=0.1, kd=45.0, target_height=0.3, 
                 i_range=(-0.1, 0.1), control_range=(1000, 1700), midpoint=1400):
        # Tuning parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_height = target_height
        self.i_range = i_range
        self.control_range = control_range
        self.midpoint = midpoint
        
        # Internal state
        self.reset()
        
    def reset(self):
        """Reset the controller state"""
        self._previous_error = 0
        self._integral = 0
        self._previous_height = 0
        
    def step(self, current_height, time_elapsed):
        """
        Calculate throttle value based on current height and time elapsed
        
        Args:
            current_height: Current height of the drone in meters
            time_elapsed: Time since last update in seconds
            
        Returns:
            throttle: Calculated throttle value (1000-2000)
        """
        # Calculate height change rate (velocity)
        height_change_rate = (current_height - self._previous_height) / max(time_elapsed, 0.0001)
        
        # Emergency braking if rising too fast or too high
        if current_height > self.target_height * 1.5 or height_change_rate > 0.05:
            self._previous_height = current_height
            return 1270
        
        # Calculate error
        error = self.target_height - current_height
        
        # Smooth error to reduce sharp changes
        smoothed_error = error * 0.8 + self._previous_error * 0.2
        
        # Calculate integral component with time consideration
        self._integral = self._integral + error * time_elapsed * 0.5
        self._integral = max(self.i_range[0], min(self._integral, self.i_range[1]))
        
        # Calculate derivative component
        derivative = (smoothed_error - self._previous_error) / max(time_elapsed, 0.0001)
        
        # Calculate PID components
        p_term = self.kp * smoothed_error
        i_term = self.ki * self._integral
        d_term = self.kd * derivative
        
        # Add rate damping to reduce oscillations
        rate_term = -100 * height_change_rate
        
        # Adaptive control based on position and velocity
        if current_height > self.target_height:
            # If above target
            if height_change_rate > 0:
                # If still rising - aggressive braking
                base_throttle = 1250
                throttle_adjustment = p_term + i_term + d_term + rate_term
                throttle = base_throttle + int(throttle_adjustment * 0.1)
                
                # Additional braking when rising
                throttle -= int(height_change_rate * 350)
            else:
                # If already descending - smooth control
                base_throttle = 1375
                throttle_adjustment = p_term + i_term + d_term + rate_term
                throttle = base_throttle + int(throttle_adjustment * 0.15)
        else:
            # If below target - smooth control
            # Adaptive base throttle depending on distance to target
            height_diff = abs(self.target_height - current_height)
            
            if height_diff > 0.01:
                # Significant deviation - stronger response
                base_throttle = 1400
                gain = 0.8
            else:
                # Minor deviation - fine tuning
                base_throttle = 1375
                gain = 0.6
                
            throttle_adjustment = p_term + i_term + d_term + rate_term
            throttle = base_throttle + int(throttle_adjustment * gain)
        
        # Limit throttle to safe range
        throttle = max(self.control_range[0], min(throttle, self.control_range[1]))
        
        # Store values for next iteration
        self._previous_error = smoothed_error
        self._previous_height = current_height
        
        return throttle
    
    def get_pid_values(self):
        """Return the current PID component values for debugging"""
        return {
            'p': self.kp * (self.target_height - self._previous_height),
            'i': self.ki * self._integral,
            'd': self.kd * ((self.target_height - self._previous_height) - self._previous_error)
        }


class PID:

    def __init__(self,

                 roll=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 roll_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 pitch=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 pitch_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 yaw=PIDaxis(0.0, 0.0, 0.0),

                 # Kv 2300 motors have midpoint 1300, Kv 2550 motors have midpoint 1250
                 throttle=PIDaxis(1,
                                  0, #0.5/height_factor * battery_factor,
                                  1,
                                  i_range=(-400, 400), control_range=(1200, 1700),
                                  d_range=(-40, 40), midpoint=1600),
                 
                 use_enhanced_throttle=False,
                 target_height=0.3):

        self.trim_controller_cap_plane = 0.05
        self.trim_controller_thresh_plane = 0.0001

        self.roll = roll
        self.roll_low = roll_low

        self.pitch = pitch
        self.pitch_low = pitch_low

        self.yaw = yaw

        self.trim_controller_cap_throttle = 5.0
        self.trim_controller_thresh_throttle = 5.0

        # Choose between standard or enhanced throttle controller
        self.use_enhanced_throttle = use_enhanced_throttle
        if use_enhanced_throttle:
            self.throttle = EnhancedThrottlePID(
                kp=90.0, 
                ki=0.1, 
                kd=45.0, 
                target_height=target_height,
                i_range=(-0.1, 0.1), 
                control_range=(1100, 1700), 
                midpoint=1400
            )
        else:
            self.throttle = throttle

        self._t = None

        # Tuning values specific to each drone
        self.roll_low.init_i = 0.0
        self.pitch_low.init_i = 0.0
        self.reset()

    def reset(self):
        """ Reset each pid and restore the initial i terms """
        # reset time variable
        self._t = None

        # reset individual PIDs
        self.roll.reset()
        self.roll_low.reset()
        self.pitch.reset()
        self.pitch_low.reset()
        self.throttle.reset()

        # restore tuning values
        self.roll_low._i = self.roll_low.init_i
        self.pitch_low._i = self.pitch_low.init_i

    def step(self, error, cmd_yaw_velocity=0):
        """ Compute the control variables from the error using the step methods
        of each axis pid.
        """
        # First time around prevent time spike
        if self._t is None:
            time_elapsed = 1
        else:
            time_elapsed = rospy.get_time() - self._t

        self._t = rospy.get_time()

        # Compute roll command
        ######################
        # if the x velocity error is within the threshold
        if abs(error.x) < self.trim_controller_thresh_plane:
            # pass the high rate i term off to the low rate pid
            self.roll_low._i += self.roll._i
            self.roll._i = 0
            # set the roll value to just the output of the low rate pid
            cmd_r = self.roll_low.step(error.x, time_elapsed)
        else:
            if error.x > self.trim_controller_cap_plane:
                self.roll_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.x < -self.trim_controller_cap_plane:
                self.roll_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.roll_low.step(error.x, time_elapsed)

            cmd_r = self.roll_low._i + self.roll.step(error.x, time_elapsed)

        # Compute pitch command
        #######################
        if abs(error.y) < self.trim_controller_thresh_plane:
            self.pitch_low._i += self.pitch._i
            self.pitch._i = 0
            cmd_p = self.pitch_low.step(error.y, time_elapsed)
        else:
            if error.y > self.trim_controller_cap_plane:
                self.pitch_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.y < -self.trim_controller_cap_plane:
                self.pitch_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.pitch_low.step(error.y, time_elapsed)

            cmd_p = self.pitch_low._i + self.pitch.step(error.y, time_elapsed)

        # Compute yaw command
        cmd_y = 1500 + cmd_yaw_velocity

        # Compute throttle command - use either standard or enhanced controller
        if self.use_enhanced_throttle:
            # For enhanced throttle, we need to convert from cm to m
            # since error.z is in cm but the enhanced controller expects meters
            current_height = error.z / 100.0  # Convert error to actual height in meters
            cmd_t = self.throttle.step(current_height, time_elapsed)
            
            # Debug output
            pid_values = self.throttle.get_pid_values()
            print("%d, %.3f, %.3f, %.3f, %.3f" % (cmd_t, error.z/100.0, pid_values['p'], pid_values['i'], pid_values['d']))
        else:
            cmd_t = self.throttle.step(error.z, time_elapsed)
            print("%d, %.3f, %.3f, %.3f, %.3f" % (cmd_t, error.z, self.throttle._p, self.throttle._i, self.throttle._d))
        
        return [cmd_r, cmd_p, cmd_y, cmd_t]