#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty, Bool, Float32
import collections


class OpticalFlowNode(object):
    """
    Subscribe to the optical flow vectors and publish linear velocity as a Twist message.
    Enhanced version with data filtering and adaptive sensitivity.
    """
    def __init__(self, node_name):
        # Initialize ROS node
        rospy.init_node(node_name)
        
        # Optical flow parameters
        camera_wh = (320, 240)        
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)
        
        # Parameters for adaptive sensitivity
        self.min_altitude = 0.05  # minimum height for calculations
        self.max_altitude = 0.5   # maximum height for calculations
        self.altitude_factor = 1.0  # coefficient for height adaptation
        
        # Filtering parameters
        self.filter_size = 5  # filter buffer size
        self.x_buffer = collections.deque(maxlen=self.filter_size)
        self.y_buffer = collections.deque(maxlen=self.filter_size)
        
        # Parameters for data quality determination
        self.max_flow_threshold = 50  # maximum allowable flow value
        self.min_flow_threshold = 2    # minimum value to account for movement
        self.quality_threshold = 0.7   # data quality threshold (0-1)
        
        # Current values
        self.altitude = 0.03  # initial height slightly above ground
        self.altitude_ts = rospy.Time.now()
        self.last_quality = 1.0
        self.last_x_motion = 0.0
        self.last_y_motion = 0.0
        self.last_update_time = rospy.Time.now()
        
        # Exponential filter setup
        self.alpha = 0.3  # filter coefficient (0-1)
        
        # Setup adaptive coefficient based on height
        self.height_coefficients = {
            0.05: 1.5,   # Increase sensitivity at low altitude
            0.1: 1.2,
            0.2: 1.0,
            0.3: 0.8,
            0.4: 0.7,
            0.5: 0.6     # Decrease sensitivity at high altitude
        }
        
        # Setup publishers and subscribers directly in __init__
        # Publishers
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)
        self.quality_pub = rospy.Publisher('/pidrone/picamera/flow_quality', Float32, queue_size=1)
        
        # Subscribers
        self._sub_mv = rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors, self.motion_cb, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range, self.altitude_cb, queue_size=1)
        self._sub_imu = rospy.Subscriber('/pidrone/imu', Imu, self.imu_cb, queue_size=1)

    def motion_cb(self, msg):
        ''' 
        Process motion vectors with filtering and quality checking.
        '''
        # Get raw optical flow data
        x = np.array(msg.x)
        y = np.array(msg.y)
        
        # Check if data exists
        if len(x) == 0 or len(y) == 0:
            rospy.logwarn("Received empty optical flow data")
            return
        
        # Check for outliers (discard values that are too large)
        x = np.clip(x, -self.max_flow_threshold, self.max_flow_threshold)
        y = np.clip(y, -self.max_flow_threshold, self.max_flow_threshold)
        
        # Data quality assessment
        # Calculate percentage of "good" vectors (not too large and not too small)
        good_vectors = np.logical_and(
            np.logical_and(np.abs(x) < self.max_flow_threshold, np.abs(y) < self.max_flow_threshold),
            np.logical_or(np.abs(x) > self.min_flow_threshold, np.abs(y) > self.min_flow_threshold)
        )
        quality = np.mean(good_vectors) if len(good_vectors) > 0 else 0.0
        
        # Publish data quality
        self.quality_pub.publish(Float32(quality))
        
        # If data quality is low, use previous values
        if quality < self.quality_threshold:
            x_motion = self.last_x_motion
            y_motion = self.last_y_motion
            rospy.logdebug("Low quality optical flow data: {:.2f}, using previous values".format(quality))
        else:
            # Calculate motion based on height
            # Get adaptive coefficient based on height
            height_coeff = self.get_height_coefficient(self.altitude)
            
            # Calculate motion considering height and adaptive coefficient
            x_motion = np.sum(x) * self.flow_coeff * self.altitude * height_coeff
            y_motion = np.sum(y) * self.flow_coeff * self.altitude * height_coeff
            
            # Add values to buffer for filtering
            self.x_buffer.append(x_motion)
            self.y_buffer.append(y_motion)
            
            # Apply median filter to remove outliers
            if len(self.x_buffer) >= 3:
                x_motion = np.median(self.x_buffer)
                y_motion = np.median(self.y_buffer)
            
            # Apply exponential filter for smoothing
            x_motion = self.alpha * x_motion + (1 - self.alpha) * self.last_x_motion
            y_motion = self.alpha * y_motion + (1 - self.alpha) * self.last_y_motion
            
            # Save current values for next iteration
            self.last_x_motion = x_motion
            self.last_y_motion = y_motion
            self.last_quality = quality
        
        # Create and publish velocity message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = x_motion
        twist_msg.twist.linear.y = -y_motion  # Invert Y to match drone coordinate system
        
        # Calculate speed considering time between updates
        dt = (twist_msg.header.stamp - self.last_update_time).to_sec()
        if dt > 0:
            # Normalize speed by time
            twist_msg.twist.linear.x /= dt
            twist_msg.twist.linear.y /= dt
        
        self.last_update_time = twist_msg.header.stamp
        
        # Publish velocity message
        self.twistpub.publish(twist_msg)
        
        # Check if altitude data is recent
        duration_from_last_altitude = rospy.Time.now() - self.altitude_ts

    def altitude_cb(self, msg):
        """
        Handler for altitude data
        """
        # Check altitude data validity
        if msg.range < 0.01 or msg.range > 10.0:
            rospy.logwarn("Received invalid altitude value: {}".format(msg.range))
            return
            
        self.altitude = msg.range
        self.altitude_ts = msg.header.stamp
    
    def imu_cb(self, msg):
        """
        Handler for IMU data for rotation compensation
        """
        # Rotation compensation can be added here if needed
        pass
    
    def get_height_coefficient(self, height):
        """
        Get adaptive coefficient based on height
        """
        # Limit height to acceptable range
        height = max(min(height, self.max_altitude), self.min_altitude)
        
        # Find nearest keys in coefficient dictionary
        heights = sorted(self.height_coefficients.keys())
        
        # If height exactly matches one of the keys
        if height in heights:
            return self.height_coefficients[height]
        
        # Find nearest values for interpolation
        lower_height = max([h for h in heights if h <= height])
        upper_height = min([h for h in heights if h >= height])
        
        # Linear interpolation
        if lower_height == upper_height:
            return self.height_coefficients[lower_height]
        
        ratio = (height - lower_height) / (upper_height - lower_height)
        lower_coeff = self.height_coefficients[lower_height]
        upper_coeff = self.height_coefficients[upper_height]
        
        return lower_coeff + ratio * (upper_coeff - lower_coeff)

def main():
    import sys
    filtered_argv = rospy.myargv(argv=sys.argv)
    
    try:
        node = OpticalFlowNode('optical_flow_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
