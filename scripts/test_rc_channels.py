#!/usr/bin/env python
import sys
import time
import traceback
import rospy
from h2rMultiWii import MultiWii
from serial import SerialException
import command_values as cmds

class RCChannelTester:
    """
    Test script to diagnose channel mapping issues between MultiWii commands
    and iNav flight controller interpretation
    """
    
    def __init__(self):
        # Connect to the flight controller board
        self.board = self.getBoard()
        self.test_commands = [
            # Test 1: Default disarm command
            {"name": "DISARM", "cmd": cmds.disarm_cmd},
            
            # Test 2: Modify throttle value to see which channel it affects
            {"name": "THROTTLE_TEST", "cmd": [1500, 1500, 1500, 1200, 1000, 1000, 1000, 1000]},
            
            # Test 3: Modify roll value to see which channel it affects
            {"name": "ROLL_TEST", "cmd": [1600, 1500, 1500, 1000, 1000, 1000, 1000, 1000]},
            
            # Test 4: Modify pitch value to see which channel it affects
            {"name": "PITCH_TEST", "cmd": [1500, 1600, 1500, 1000, 1000, 1000, 1000, 1000]},
            
            # Test 5: Modify yaw value to see which channel it affects
            {"name": "YAW_TEST", "cmd": [1500, 1500, 1600, 1000, 1000, 1000, 1000, 1000]},
            
            # Test 6: Modify aux1 value to see which channel it affects
            {"name": "AUX1_TEST", "cmd": [1500, 1500, 1500, 1000, 1600, 1000, 1000, 1000]},
            
            # Additional tests for throttle mapping
            {"name": "THROTTLE_TEST_ALT1", "cmd": [1000, 1000, 1000, 1500, 1000, 1000, 1000, 1000]},
            {"name": "THROTTLE_TEST_ALT2", "cmd": [1000, 1000, 1500, 1000, 1000, 1000, 1000, 1000]},
            {"name": "THROTTLE_TEST_ALT3", "cmd": [1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000]},
            {"name": "THROTTLE_TEST_ALT4", "cmd": [1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000]},
            
            # Simple AUX1 arm test (AUX1 > 1800)
            {"name": "AUX1_ARM", "cmd": [1500, 1500, 1500, 1000, 1900, 1000, 1000, 1000]},
            
            # Simple AUX1 disarm test (AUX1 < 1500)
            {"name": "AUX1_DISARM", "cmd": [1500, 1500, 1500, 1000, 1400, 1000, 1000, 1000]},
        ]
        
    def getBoard(self):
        """ Connect to the flight controller board """
        try:
            board = MultiWii('/dev/ttyACM0')
        except SerialException as e:
            print("usb0 failed: " + str(e))
            try:
                board = MultiWii('/dev/ttyACM1')
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                sys.exit(1)
        return board
    
    def read_rc_channels(self):
        """
        Read RC channels from the flight controller and display them
        """
        try:
            self.board.getData(MultiWii.RC)
            rc_data = self.board.rcChannels
            if rc_data:
                print("\n--- RC Channels ---")
                print("Roll:     ", rc_data.get('roll', 'N/A'))
                print("Pitch:    ", rc_data.get('pitch', 'N/A'))
                print("Yaw:      ", rc_data.get('yaw', 'N/A'))
                print("Throttle: ", rc_data.get('throttle', 'N/A'))
                print("***********************")
                print("* AUX1 (5th channel): ", rc_data.get('aux1', 'N/A'), " *")
                print("***********************")
                print("-----------------\n")
                
                # Print the raw data to help with debugging
                print("Raw channel values:")
                for i, (key, value) in enumerate(rc_data.items()):
                    print(f"Channel {i+1} ({key}): {value}")
            else:
                print("Failed to get RC data")
        except Exception as e:
            print("Error reading RC channels: %s" % e)
            print(traceback.format_exc())
    
    def read_arm_status(self):
        """Read the arm status from the flight controller"""
        try:
            self.board.getData(MultiWii.STATUS)
            armed = self.board.status['armed']
            print(f"\n*** ARMED STATUS: {'ARMED' if armed else 'DISARMED'} ***\n")
            return armed
        except Exception as e:
            print(f"Error reading arm status: {e}")
            print(traceback.format_exc())
            return None
    
    def send_command_and_read_response(self, command_dict):
        """Send a command to the flight controller and read the response"""
        cmd = command_dict["cmd"]
        name = command_dict["name"]
        
        print(f"\n=== SENDING {name} COMMAND ===")
        print(f"Command values: {cmd}")
        print(f"Expected mapping: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]")
        print(f"Command indices: [ 0,    1,    2,     3,     4,    5,    6,    7 ]")
        
        try:
            # Read arm status before sending command
            print("Before command:")
            before_armed = self.read_arm_status()
            
            # Send the command
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmd)
            self.board.receiveDataPacket()
            time.sleep(0.5)  # Give time for the command to be processed
            
            # Read back the current RC channel values
            self.read_rc_channels()
            
            # Read arm status after sending command
            print("After command:")
            after_armed = self.read_arm_status()
            
            # Check if arm status changed
            if before_armed is not None and after_armed is not None:
                if before_armed != after_armed:
                    if after_armed:
                        print("\n!!! ARMING DETECTED - This command ARMED the drone !!!")
                    else:
                        print("\n!!! DISARMING DETECTED - This command DISARMED the drone !!!")
                else:
                    print("\nNo change in arm status.")
            
        except Exception as e:
            print(f"Error sending {name} command: {e}")
            print(traceback.format_exc())
    
    def run_single_test(self, test_name):
        """Run a single test by name"""
        for test in self.test_commands:
            if test["name"] == test_name:
                self.send_command_and_read_response(test)
                return True
        print(f"Test {test_name} not found!")
        return False
    
    def run_arm_disarm_test(self):
        """Run a combined arm and disarm test sequence"""
        print("\n=== RUNNING ARM/DISARM TEST SEQUENCE ===")
        
        # First make sure we're disarmed
        print("\nStep 1: Ensuring drone is disarmed...")
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        time.sleep(1)
        
        # Check initial status
        print("\nInitial status:")
        initial_armed = self.read_arm_status()
        if initial_armed:
            print("WARNING: Drone is still armed after sending disarm command!")
            return
            
        # Try to arm
        print("\nStep 2: Testing arm command (AUX1 > 1800)...")
        arm_cmd = [1500, 1500, 1500, 1000, 1900, 1000, 1000, 1000]
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, arm_cmd)
        self.board.receiveDataPacket()
        time.sleep(1)
        
        # Check if armed
        mid_armed = self.read_arm_status()
        if not mid_armed:
            print("WARNING: Failed to arm the drone with AUX1 > 1800")
            print("Try testing different AUX channels or values")
        else:
            print("SUCCESS: Drone armed successfully with AUX1 > 1800")
        
        # Try to disarm
        print("\nStep 3: Testing disarm command (AUX1 < 1500)...")
        disarm_cmd = [1500, 1500, 1500, 1000, 1400, 1000, 1000, 1000]
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
        self.board.receiveDataPacket()
        time.sleep(1)
        
        # Check final status
        final_armed = self.read_arm_status()
        if final_armed:
            print("WARNING: Failed to disarm the drone with AUX1 < 1500")
            print("Try testing different AUX channels or values")
            
            # Force disarm for safety
            print("\nForcing disarm with default command...")
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
            self.board.receiveDataPacket()
        else:
            print("SUCCESS: Drone disarmed successfully with AUX1 < 1500")
            
        print("\n=== ARM/DISARM TEST COMPLETE ===")
    
    def run_tests(self):
        """Run through all test commands"""
        print("\n*** RC CHANNEL MAPPING TEST ***")
        print("This test will help diagnose channel mapping issues between")
        print("the commands being sent and how they're received by the flight controller.")
        print("\nWARNING: Ensure props are removed and the drone is secured!")
        
        print("\nOptions:")
        print("1. Run all tests")
        print("2. Run specific test")
        print("3. Run throttle tests only")
        print("4. Test AUX1 arm (value > 1800)")
        print("5. Test AUX1 disarm (value < 1500)")
        print("6. Run combined arm/disarm test")
        print("7. Quit")
        
        choice = input("\nEnter your choice (1-7): ")
        
        if choice == '1':
            print("\nRunning all tests...")
            for test_cmd in self.test_commands:
                self.send_command_and_read_response(test_cmd)
                time.sleep(1)  # Pause between tests
        elif choice == '2':
            print("\nAvailable tests:")
            for i, test in enumerate(self.test_commands):
                print(f"{i+1}. {test['name']}")
            test_idx = int(input("\nEnter test number: ")) - 1
            if 0 <= test_idx < len(self.test_commands):
                self.send_command_and_read_response(self.test_commands[test_idx])
            else:
                print("Invalid test number!")
        elif choice == '3':
            print("\nRunning throttle tests...")
            throttle_tests = [test for test in self.test_commands if 'THROTTLE' in test['name']]
            for test in throttle_tests:
                self.send_command_and_read_response(test)
                time.sleep(1)
        elif choice == '4':
            print("\nTesting AUX1 arm (value > 1800)...")
            # Find the AUX1_ARM test
            arm_test = next((test for test in self.test_commands if test["name"] == "AUX1_ARM"), None)
            if arm_test:
                self.send_command_and_read_response(arm_test)
                # Always disarm after arm test for safety
                time.sleep(1)
                self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
                self.board.receiveDataPacket()
        elif choice == '5':
            print("\nTesting AUX1 disarm (value < 1500)...")
            # Find the AUX1_DISARM test
            disarm_test = next((test for test in self.test_commands if test["name"] == "AUX1_DISARM"), None)
            if disarm_test:
                self.send_command_and_read_response(disarm_test)
        elif choice == '6':
            self.run_arm_disarm_test()
        elif choice == '7':
            print("Exiting...")
            return
        else:
            print("Invalid choice!")
            return
            
        print("\n*** TEST COMPLETE ***")
        print("Based on the results, you should be able to determine:")
        print("1. Which command index maps to which actual channel")
        print("2. If there's a mismatch in the expected channel order")
        print("3. Which AUX channel controls arming/disarming")
        print("\nIf the throttle channel is not at the expected position (index 3),")
        print("you'll need to adjust your command_values.py to match the actual mapping.")
        print("\nFor example, if throttle is actually at index 2 and yaw is at index 3,")
        print("use the fix_command_values.py script with mapping: 0,1,3,2,4,5,6,7")

def main():
    # Initialize ROS node
    rospy.init_node('rc_channel_tester', anonymous=True)
    
    tester = RCChannelTester()
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        print("\nTest aborted by user")
    except Exception as e:
        print(f"Error during test: {e}")
        print(traceback.format_exc())
    finally:
        # Always send disarm command when done
        print("\nSending DISARM command")
        tester.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        tester.board.receiveDataPacket()

if __name__ == '__main__':
    main() 