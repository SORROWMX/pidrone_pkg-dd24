#!/usr/bin/env python
import traceback
import sys
import yaml

import rospy
import rospkg
import signal
import numpy as np

print("tf import")
import tf
print("done tf import")


import command_values as cmds
from sensor_msgs.msg import Imu
from h2rMultiWii import MultiWii
from serial import SerialException
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import Quaternion
from pidrone_pkg.msg import Mode, RC, State
from sensor_msgs.msg import Range
import os


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/range
    /pidrone/heartbeat/web_interface
    /pidrone/heartbeat/pid_controller
    /pidrone/state
    """

    def __init__(self):
        # Connect to the flight controller board
        print("getboard")
        self.board = self.getBoard()
        print("done")
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'         #initialize as disarmed
        self.prev_mode = 'DISARMED'         #initialize as disarmed
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd      #initialize as disarmed
        self.last_command = cmds.disarm_cmd
        # store the mode publisher
        self.modepub = None
        # store the time for angular velocity calculations
        self.time = rospy.Time.now()

        # Initialize the Imu Message
        ############################
        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()

        self.imu_message = Imu()
        self.imu_message.header = header

        # Accelerometer parameters
        ##########################
        print("loading")
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/multiwii.yaml" % path) as f:
            means = yaml.load(f)
        print("done")
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX = means["ax"] * self.accRawToMss
        self.accZeroY = means["ay"] * self.accRawToMss
        self.accZeroZ = means["az"] * self.accRawToMss


    # ROS subscriber callback methods:
    ##################################
    def desired_mode_callback(self, msg):
        """ Set the current mode to the desired mode """
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        self.update_command()

    def fly_commands_callback(self, msg):
        """ Store and send the flight commands if the current mode is FLYING """
        if self.curr_mode == 'FLYING':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            # Format: [roll, pitch, throttle, yaw] to match AETR map in INAV
            self.command = [r, p, t, y] + cmds.idle_cmd[4:8]


    # Update methods:
    #################
    def update_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """
        try:
            # extract roll, pitch, heading
            attitude_data = self.board.getData(MultiWii.ATTITUDE)
            # extract lin_acc_x, lin_acc_y, lin_acc_z
            imu_data = self.board.getData(MultiWii.RAW_IMU)

            # If no data received, skip this update
            if attitude_data is None or imu_data is None:
                print("Failed to get IMU data, skipping update")
                return

            # calculate values to update imu_message:
            roll = np.deg2rad(self.board.attitude['angx'])
            pitch = -np.deg2rad(self.board.attitude['angy'])
            heading = np.deg2rad(self.board.attitude['heading'])
            # Note that at pitch angles near 90 degrees, the roll angle reading can
            # fluctuate a lot
            # transform heading (similar to yaw) to standard math conventions, which
            # means angles are in radians and positive rotation is CCW
            heading = (-heading) % (2 * np.pi)
            # When first powered up, heading should read near 0
            # get the previous roll, pitch, heading values
            previous_quaternion = self.imu_message.orientation
            quaternion_array = [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
            previous_roll, previous_pitch, previous_heading = tf.transformations.euler_from_quaternion(quaternion_array)

            # Although quaternion_from_euler takes a heading in range [0, 2pi),
            # euler_from_quaternion returns a heading in range [0, pi] or [0, -pi).
            # Thus need to convert the returned heading back into the range [0, 2pi).
            previous_heading = previous_heading % (2 * np.pi)

            # transform euler angles into quaternion
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
            # calculate the linear accelerations
            lin_acc_x = self.board.rawIMU['ax'] * self.accRawToMss - self.accZeroX
            lin_acc_y = self.board.rawIMU['ay'] * self.accRawToMss - self.accZeroY
            lin_acc_z = self.board.rawIMU['az'] * self.accRawToMss - self.accZeroZ

            # Rotate the IMU frame to align with our convention for the drone's body
            # frame. IMU: x is forward, y is left, z is up. We want: x is right,
            # y is forward, z is up.
            lin_acc_x_drone_body = -lin_acc_y
            lin_acc_y_drone_body = lin_acc_x
            lin_acc_z_drone_body = lin_acc_z

            # Account for gravity's affect on linear acceleration values when roll
            # and pitch are nonzero. When the drone is pitched at 90 degrees, for
            # example, the z acceleration reads out as -9.8 m/s^2. This makes sense,
            # as the IMU, when powered up / when the calibration script is called,
            # zeros the body-frame z-axis acceleration to 0, but when it's pitched
            # 90 degrees, the body-frame z-axis is perpendicular to the force of
            # gravity, so, as if the drone were in free-fall (which was roughly
            # confirmed experimentally), the IMU reads -9.8 m/s^2 along the z-axis.
            g = 9.8
            lin_acc_x_drone_body = lin_acc_x_drone_body + g*np.sin(roll)*np.cos(pitch)
            lin_acc_y_drone_body = lin_acc_y_drone_body + g*np.cos(roll)*(-np.sin(pitch))
            lin_acc_z_drone_body = lin_acc_z_drone_body + g*(1 - np.cos(roll)*np.cos(pitch))

            # calculate the angular velocities of roll, pitch, and yaw in rad/s
            time = rospy.Time.now()
            dt = time.to_sec() - self.time.to_sec()
            dr = roll - previous_roll
            dp = pitch - previous_pitch
            dh = heading - previous_heading
            angvx = self.near_zero(dr / dt)
            angvy = self.near_zero(dp / dt)
            angvz = self.near_zero(dh / dt)
            self.time = time

            # Update the imu_message:
            # header stamp
            self.imu_message.header.stamp = time
            # orientation
            self.imu_message.orientation.x = quaternion[0]
            self.imu_message.orientation.y = quaternion[1]
            self.imu_message.orientation.z = quaternion[2]
            self.imu_message.orientation.w = quaternion[3]
            # angular velocities
            self.imu_message.angular_velocity.x = angvx
            self.imu_message.angular_velocity.y = angvy
            self.imu_message.angular_velocity.z = angvz
            # linear accelerations
            self.imu_message.linear_acceleration.x = lin_acc_x_drone_body
            self.imu_message.linear_acceleration.y = lin_acc_y_drone_body
            self.imu_message.linear_acceleration.z = lin_acc_z_drone_body
        except Exception as e:
            print("Error updating IMU message: {}".format(e))
            # Don't raise exception to continue program execution

    def update_command(self):
        ''' Set command values if the mode is ARMED or DISARMED '''
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd

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
            else:
                print("Failed to get RC data")
        except Exception as e:
            print("Error reading RC channels: %s" % e)
            print(traceback.format_exc())

    # Helper Methods:
    #################
    def getBoard(self):
        """ Connect to the flight controller board """
        # (if the flight cotroll usb is unplugged and plugged back in,
        #  it becomes .../USB1)
        try:
            board = MultiWii('/dev/ttyACM0')
        except SerialException as e:
            print(("usb0 failed: " + str(e)))
            try:
                board = MultiWii('/dev/ttyACM1')
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                raise
                sys.exit()
        return board

    def send_rc_cmd(self):
        """ Send commands to the flight controller board """
        assert len(self.command) is 8, "COMMAND HAS WRONG SIZE, expected 8, got "+str(len(self.command))
        try:
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, self.command)
            result = self.board.receiveDataPacket()
            
            # If result is None, an error occurred while reading the response,
            # but this is not critical for sending the command
            if result is None:
                print("Warning: Did not receive confirmation for command")
            
            if (self.command != self.last_command):
                print('new command sent:', self.command)
                self.last_command = self.command
                
        except Exception as e:
            print("Error sending RC command: {}".format(e))
            # Don't raise exception to continue program execution

    def near_zero(self, n):
        """ Set a number to zero if it is below a threshold value """
        return 0 if abs(n) < 0.0001 else n

    def ctrl_c_handler(self, signal, frame):
        """ Disarm the drone and quits the flight controller node """
        print("\nCaught ctrl-c! About to Disarm!")
        self.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)
        self.modepub.publish('DISARMED')
        print("Successfully Disarmed")
        sys.exit()

    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()
        self.range = msg.range

    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat"""
        self.heartbeat_state_estimator = rospy.Time.now()

    def shouldIDisarm(self):
        """
        Check for missing heartbeats and issue warning for high altitude
        """
        curr_time = rospy.Time.now()
        disarm = False
        if curr_time - self.heartbeat_pid_controller > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving flight commands.')
            print('Check the pid_controller node\n')
            disarm = True
        if curr_time - self.heartbeat_infrared > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving data from the IR sensor.')
            print('Check the infrared node\n')
            disarm = True

        # Instead of disarming at high altitude, just issue a warning
        if self.range > 0.5:
            print(('\nWarning: High altitude detected: ' + str(self.range) + 'm'))
            # Don't disarm the drone, just warn the user
            
        if curr_time - self.heartbeat_state_estimator > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving a state estimate.')
            print('Check the state_estimator node\n')
            disarm = True

        return disarm


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    print("init")
    rospy.init_node(node_name)
    print("done")
    # create the FlightController object
    fc = FlightController()
    curr_time = rospy.Time.now()
    fc.heartbeat_infrared = curr_time
    fc.range = None
    fc.heartbeat_web_interface= curr_time
    fc.heartbeat_pid_controller = curr_time
    fc.heartbeat_flight_controller = curr_time
    fc.heartbeat_state_estimator = curr_time

    # Publishers
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    print('Publishing:')
    print('/pidrone/imu')
    print('/pidrone/mode')

    # Subscribers
    ############
    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback)
    # heartbeat subscribers
    rospy.Subscriber("/pidrone/range", Range, fc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, fc.heartbeat_web_interface_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, fc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/state", State, fc.heartbeat_state_estimator_callback)


    # signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    # set the loop rate (Hz)
    r = rospy.Rate(60)
    try:
        while not rospy.is_shutdown():
            # if the current mode is anything other than disarmed
            # preform as safety check
                # Break the loop if a safety check has failed
            if fc.shouldIDisarm():
                print("mode", fc.curr_mode)
                break
                
            # update and publish flight controller readings
            fc.update_imu_message()
            imupub.publish(fc.imu_message)

            # update and send the flight commands to the board
            fc.update_command()
            fc.send_rc_cmd()

            # publish the current mode of the drone
            fc.modepub.publish(fc.curr_mode)

            # sleep for the remainder of the loop time
            r.sleep()
            
    except SerialException:
        print('\nCannot connect to the flight controller board.')
        print('The USB is unplugged. Please check connection.')
    except Exception as e:
        print('there was an internal error', e)
        print(traceback.format_exc())
    finally:
        print('Shutdown received')
        print('Sending DISARM command')
        fc.board.send_raw_command(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        fc.board.receiveDataPacket()


if __name__ == '__main__':
    main()
