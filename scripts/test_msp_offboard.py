#!/usr/bin/env python

import rospy
import time
import sys
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Empty, Bool

class MSPOffboardTester:
    """
    Class for testing MSP Offboard
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('msp_offboard_tester')
        
        # Wait for services to become available
        rospy.loginfo("Waiting for MSP Offboard services...")
        rospy.wait_for_service('/pidrone/msp_offboard/takeoff')
        rospy.wait_for_service('/pidrone/msp_offboard/land')
        rospy.wait_for_service('/pidrone/msp_offboard/navigate')
        rospy.wait_for_service('/pidrone/msp_offboard/set_position')
        rospy.wait_for_service('/pidrone/msp_offboard/set_velocity')
        
        # Create service proxies
        self.takeoff_service = rospy.ServiceProxy('/pidrone/msp_offboard/takeoff', Trigger)
        self.land_service = rospy.ServiceProxy('/pidrone/msp_offboard/land', Trigger)
        self.navigate_service = rospy.ServiceProxy('/pidrone/msp_offboard/navigate', Trigger)
        self.set_position_service = rospy.ServiceProxy('/pidrone/msp_offboard/set_position', Trigger)
        self.set_velocity_service = rospy.ServiceProxy('/pidrone/msp_offboard/set_velocity', Trigger)
        
        # Subscribe to monitoring topics
        self.height_stable = False
        rospy.Subscriber('/pidrone/height_stable', Bool, self.height_stable_callback)
        
        rospy.loginfo("MSP Offboard Tester ready")
    
    def height_stable_callback(self, msg):
        """Handler for height stability messages"""
        self.height_stable = msg.data
    
    def wait_for_height_stabilization(self, timeout=10):
        """Wait for height to stabilize"""
        start_time = time.time()
        stable_count = 0
        
        while stable_count < 5 and time.time() - start_time < timeout:
            if self.height_stable:
                stable_count += 1
            else:
                stable_count = 0
            
            time.sleep(0.1)
        
        if stable_count >= 5:
            rospy.loginfo("Height stabilized")
            return True
        else:
            rospy.logwarn("Height stabilization timeout")
            return False
    
    def test_takeoff_and_land(self):
        """Test takeoff and landing"""
        rospy.loginfo("Starting takeoff and landing test")
        
        # Takeoff
        rospy.loginfo("Takeoff...")
        response = self.takeoff_service()
        if not response.success:
            rospy.logerr("Takeoff error: {}".format(response.message))
            return False
        
        # Wait for height to stabilize
        if not self.wait_for_height_stabilization(timeout=20):
            rospy.logerr("Failed to stabilize height after takeoff")
            self.land_service()
            return False
        
        rospy.loginfo("Drone successfully took off and stabilized")
        rospy.sleep(3.0)
        
        # Landing
        rospy.loginfo("Landing...")
        response = self.land_service()
        if not response.success:
            rospy.logerr("Landing error: {}".format(response.message))
            return False
        
        rospy.loginfo("Takeoff and landing test completed successfully")
        return True
    
    def test_square_flight(self, side_length=0.3):
        """Test flight in a square pattern"""
        rospy.loginfo("Starting square flight test (side length {} m)".format(side_length))
        
        # Takeoff
        rospy.loginfo("Takeoff...")
        response = self.takeoff_service()
        if not response.success:
            rospy.logerr("Takeoff error: {}".format(response.message))
            return False
        
        # Wait for height to stabilize
        if not self.wait_for_height_stabilization(timeout=20):
            rospy.logerr("Failed to stabilize height after takeoff")
            self.land_service()
            return False
        
        rospy.loginfo("Drone successfully took off and stabilized")
        rospy.sleep(3.0)
        
        # Square flight
        waypoints = [
            {"x": side_length, "y": 0, "z": 0, "frame_id": "body"},
            {"x": 0, "y": side_length, "z": 0, "frame_id": "body"},
            {"x": -side_length, "y": 0, "z": 0, "frame_id": "body"},
            {"x": 0, "y": -side_length, "z": 0, "frame_id": "body"}
        ]
        
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo("Flying to point {}: x={}, y={}".format(i+1, waypoint['x'], waypoint['y']))
            
            # Send movement command
            response = self.set_position_service(
                x=waypoint['x'],
                y=waypoint['y'],
                z=waypoint['z'],
                yaw=0.0,
                frame_id=waypoint['frame_id'],
                auto_arm=False
            )
            
            if not response.success:
                rospy.logerr("Error flying to point {}: {}".format(i+1, response.message))
                self.land_service()
                return False
            
            # Wait for arrival at point
            rospy.sleep(5.0)
        
        # Return to starting point
        rospy.loginfo("Returning to starting point")
        response = self.set_position_service(
            x=0.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            frame_id="map",
            auto_arm=False
        )
        
        if not response.success:
            rospy.logerr("Error returning to start: {}".format(response.message))
            self.land_service()
            return False
        
        # Wait for arrival at starting point
        rospy.sleep(5.0)
        
        # Landing
        rospy.loginfo("Landing...")
        response = self.land_service()
        if not response.success:
            rospy.logerr("Landing error: {}".format(response.message))
            return False
        
        rospy.loginfo("Square flight test completed successfully")
        return True
    
    def test_velocity_control(self):
        """Test velocity control"""
        rospy.loginfo("Starting velocity control test")
        
        # Takeoff
        rospy.loginfo("Takeoff...")
        response = self.takeoff_service()
        if not response.success:
            rospy.logerr("Takeoff error: {}".format(response.message))
            return False
        
        # Wait for height to stabilize
        if not self.wait_for_height_stabilization(timeout=20):
            rospy.logerr("Failed to stabilize height after takeoff")
            self.land_service()
            return False
        
        rospy.loginfo("Drone successfully took off and stabilized")
        rospy.sleep(3.0)
        
        # Forward movement
        rospy.loginfo("Moving forward at 0.2 m/s for 3 seconds")
        response = self.set_velocity_service(
            vx=0.0,
            vy=0.2,
            vz=0.0,
            yaw_rate=0.0,
            frame_id="body",
            auto_arm=False
        )
        
        if not response.success:
            rospy.logerr("Error setting velocity: {}".format(response.message))
            self.land_service()
            return False
        
        rospy.sleep(3.0)
        
        # Stop
        rospy.loginfo("Stopping")
        response = self.set_velocity_service(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            frame_id="body",
            auto_arm=False
        )
        
        if not response.success:
            rospy.logerr("Error stopping: {}".format(response.message))
            self.land_service()
            return False
        
        rospy.sleep(2.0)
        
        # Right movement
        rospy.loginfo("Moving right at 0.2 m/s for 3 seconds")
        response = self.set_velocity_service(
            vx=0.2,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            frame_id="body",
            auto_arm=False
        )
        
        if not response.success:
            rospy.logerr("Error setting velocity: {}".format(response.message))
            self.land_service()
            return False
        
        rospy.sleep(3.0)
        
        # Stop
        rospy.loginfo("Stopping")
        response = self.set_velocity_service(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            frame_id="body",
            auto_arm=False
        )
        
        if not response.success:
            rospy.logerr("Error stopping: {}".format(response.message))
            self.land_service()
            return False
        
        rospy.sleep(2.0)
        
        # Landing
        rospy.loginfo("Landing...")
        response = self.land_service()
        if not response.success:
            rospy.logerr("Landing error: {}".format(response.message))
            return False
        
        rospy.loginfo("Velocity control test completed successfully")
        return True


def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage: test_msp_offboard.py [takeoff_land|square|velocity]")
        return
    
    test_type = sys.argv[1]
    
    # Create tester
    tester = MSPOffboardTester()
    
    # Run selected test
    if test_type == "takeoff_land":
        tester.test_takeoff_and_land()
    elif test_type == "square":
        side_length = 0.3
        if len(sys.argv) > 2:
            try:
                side_length = float(sys.argv[2])
            except ValueError:
                print("Invalid side length: {}".format(sys.argv[2]))
                return
        
        tester.test_square_flight(side_length)
    elif test_type == "velocity":
        tester.test_velocity_control()
    else:
        print("Unknown test type: {}".format(test_type))
        print("Available types: takeoff_land, square, velocity")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 