#!/usr/bin/env python3
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/arpit6232/Leica_Total_Station_ROS

    For more info on the documentation of TS16 & GEOCOM API, go to the following links: 
    1. https://www.naic.edu/~phil/hardware/theodolites/FlexLine_GeoCOM_Manual_en.pdf
    2. https://secure.fltgeosystems.com/uploads/tips/documents/244-1493928689.pdf
"""

# Dependencies
import rclpy
from rclpy.node import Node
import numpy as np
import time
import serial
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from ament_index_python.packages import get_package_share_directory


class ts16_localizer(Node):

    def __init__(self):
        """
        Initialize the node, open serial port
        """
        # Initialize node
        super().__init__('total_station')

        self.declare_parameter('total_station_hz', 10)
        self.total_station_hz = self.get_parameter(
            'total_station_hz').get_parameter_value().integer_value
        self.timer = self.create_timer(
            1 / self.total_station_hz, self.total_station_timer_callback)

        self.declare_parameter('total_station_verbose', False)
        self.total_station_verbose = self.get_parameter(
            'total_station_verbose').get_parameter_value().bool_value

        self.declare_parameter('positional_rms', 0.0)
        self.positional_rms = self.get_parameter(
            'positional_rms').get_parameter_value().double_value

        self.TS_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "total_station_prism", 10)

        self.declare_parameter('total_station_port', 'ttyUSB0')
        self.total_station_port = self.get_parameter(
            'total_station_port').get_parameter_value().string_value

        self.declare_parameter('total_station_baudrate', 19200)
        self.total_station_baudate = self.get_parameter(
            'total_station_baudrate').get_parameter_value().integer_value

        self.declare_parameter('ts_prism_x_offset', 0.0)
        self.ts_prism_x_offset = self.get_parameter(
            'ts_prism_x_offset').get_parameter_value().double_value

        self.declare_parameter('ts_prism_y_offset', 0.0)
        self.ts_prism_y_offset = self.get_parameter(
            'ts_prism_y_offset').get_parameter_value().double_value

        self.declare_parameter('ts_prism_z_offset', 0.0)
        self.ts_prism_z_offset = self.get_parameter(
            'ts_prism_z_offset').get_parameter_value().double_value

        self.get_logger().info(f"x Offset: {self.ts_prism_x_offset}")
        self.get_logger().info(f"y Offset: {self.ts_prism_y_offset}")
        self.get_logger().info(f"z Offset: {self.ts_prism_z_offset}")


        self.declare_parameter('ts_prism_z_rotation', 0.0)
        self.ts_prism_z_rotation = self.get_parameter(
            'ts_prism_z_rotation').get_parameter_value().double_value

        self.z_rot_mat = np.array([
            [np.cos(self.ts_prism_z_rotation), -
             np.sin(self.ts_prism_z_rotation), 0],
            [np.sin(self.ts_prism_z_rotation), np.cos(
                self.ts_prism_z_rotation), 0],
            [0, 0, 1]], dtype=np.double)        

        self.get_logger().info(f"Port: {self.total_station_port}")
        self.get_logger().info(f"Baudrate: {self.total_station_baudate}")

        try:
            self.serialPortLeicaTS16 = serial.Serial(
                port=self.total_station_port,
                baudrate=self.total_station_baudate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
        except:
            self.serialPortLeicaTS16 = None
            self.get_logger().info("Could not connect to port")

        # Open Serial Connection to TS16
        if (not self.open_ts_connection(nRetries=5)):
            self.get_logger().info("Failed to Connect with Leica TS16")

    def open_ts_connection(self, nRetries=5):
        """
        Open Serial Connection to LeicaTS16
        """

        try:

            tries = 0
            while (not self.serialPortLeicaTS16.isOpen() and tries <= nRetries):
                self.serialPortLeicaTS16.open()
                if (not self.serialPortLeicaTS16.isOpen()):
                    self.serialPortLeicaTS16.close()
                    time.sleep(0.5)
                    tries += 1

            if (not self.serialPortLeicaTS16.isOpen()):
                self.get_logger().info('Problem opening port')

            return self.serialPortLeicaTS16.isOpen()

        except:
            self.get_logger().info("Connection Error - Leica TS not connected?")
            return False

    def close_ts_connection(self):
        """
        Close Serial Connection to Leica TS16
        """

        self.serialPortLeicaTS16.close()

        if (not self.serialPortLeicaTS16.isOpen()):
            print('Problem closing TS16 port')

        return not self.serialPortLeicaTS16.isClose()

    def create_geocom_request(self, cmd, args):
        """
        Create a GEOCOM formatted request
        See format here: (pg. 128 for ASCII-Request example)
        https://www.naic.edu/~phil/hardware/theodolites/TPS1200_GeoCOM_Manual.pdf
        """

        request = '%R1Q,'
        request = request + str(cmd)
        request = request + ':'

        if (len(args) > 0):
            for i in range(0, len(args)-1):
                request = request + str(args[i])
                request = request + ','

            request = request + str(args[-1])

        return request + '\r\n'

    def take_ts_measurement(self):
        """
        Request a new measurement be taken from TS
        """

        TMC_MEASURE_MODE = 1  # Default Distance-measurement
        TMC_DoMeasurement_CMD = '2008'
        TMC_AUTO_INC_MODE = 1
        request = self.create_geocom_request(
            TMC_DoMeasurement_CMD, [TMC_MEASURE_MODE, TMC_AUTO_INC_MODE])
        encoded_req = request.encode()

        # Send encoded command
        self.serialPortLeicaTS16.write(encoded_req)
        t_start = time.time()

        length = 10
        t_timeout = 0.5
        while ((self.serialPortLeicaTS16.inWaiting() < length or (length == 0 and self.serialPortLeicaTS16.inWaiting() == 0)) and time.time()-t_start < t_timeout):
            time.sleep(0.001)
        time.sleep(0.025)

        serial_output = self.serialPortLeicaTS16.readline()
        return "".join(chr(x) for x in bytearray(serial_output))

    def get_ts_coordinate(self):
        """
        Get current measurement from TS
        """

        waitTime = 100  # Delay getting the measurement
        TMC_GetCoordinate_CMD = '2082'
        TMC_AUTO_INC_MODE = 1
        request = self.create_geocom_request(
            TMC_GetCoordinate_CMD, [waitTime, TMC_AUTO_INC_MODE])
        encoded_req = request.encode()

        # Send encoded command
        self.serialPortLeicaTS16.write(encoded_req)
        t_start = time.time()

        length = 100
        t_timeout = 3
        while ((self.serialPortLeicaTS16.inWaiting() < length or (length == 0 and self.serialPortLeicaTS16.inWaiting() == 0)) and time.time()-t_start < t_timeout):
            time.sleep(0.001)
        time.sleep(0.025)

        serial_output = self.serialPortLeicaTS16.readline()
        return "".join(chr(x) for x in bytearray(serial_output))

    def total_station_timer_callback(self):
        """
        Requesting and reading total station data on regular intervals
        """

        if self.serialPortLeicaTS16 == None:
            # self.get_logger().info("Failed callback, no serial connection")
            return

        # Get coordinate from new measurement
        coordinate_output = self.get_ts_coordinate()

        # Parse coordinate from new measurement if possible
        position = self.parse_coordinate_output(coordinate_output)

        # If position can be correctly parsed, publish!
        if len(position) == 3:
            self.get_logger().info("Publishing position from TS")
            success = self.publishTSPose(position, "map", self.TS_publisher)
            if not success:
                self.get_logger().info(
                    f"Failed Output String Parse: {coordinate_output}")
        else:
            self.get_logger().info(
                f"Failed Output String: {coordinate_output}")

        return

    def parse_coordinate_output(self, serial_output, continuous=False):

        try:
            response_high_level_split = serial_output.split(":")

            response_parts = response_high_level_split[1].split(",")

            return_code = int(response_parts[0])
            if return_code not in [0, 1283, 1284]:
                self.get_logger().info(
                    f"TS request execution unsuccessful, error code: {return_code}")
                return []

            # Position in "Easting-Northing-Height" format
        # Position in "Easting-Northing-Height" format
            # Position in "Easting-Northing-Height" format
            # Equivalent of X-Y-Z when coordinate system setup appropriately
            enh_position_coordinate = response_parts[1:4]
            enh_position_coordinate_continuous = response_parts[5:8]

            return enh_position_coordinate if continuous else enh_position_coordinate_continuous

        except:
            return []

    def publishTSPose(self, pos, frame_id, publisher, transform=True):
        """
        Publish Position using provided publisher
        :param: [Double] PoseData, String frame_id
        :returns: none
        """

        try:

            # Transform point into site-map frame
            transformed_pos = [float(num) for num in pos]

            # Attempt to sanitize input
            # transformed_pos = [num.split("\x00")[0] for num in pos]

            if transform:
                transformed_pos = (self.z_rot_mat @ np.array(transformed_pos).T).tolist()

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.pose.pose.position.x = float(transformed_pos[0]) + self.ts_prism_x_offset
            pose_msg.pose.pose.position.y = float(transformed_pos[1]) + self.ts_prism_y_offset
            pose_msg.pose.pose.position.z = float(transformed_pos[2]) + self.ts_prism_z_offset
            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = 0.0
            pose_msg.pose.pose.orientation.w = 1.0
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = frame_id

            # Assuming diagonal covariance matrix
            # Setting rotation covariance to 1 by default, but should be thrown out during sensor fusion
            pose_msg.pose.covariance = [self.positional_rms, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, self.positional_rms, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, self.positional_rms, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            publisher.publish(pose_msg)
            return True
        except:
            self.get_logger().info(f"Could not publish position: {pos}")
            return False


def main(args=None):
    """
    Entrance to TS Node
    """

    rclpy.init(args=args)

    ts16 = ts16_localizer()
    ts16.open_ts_connection(nRetries=5)

    rclpy.spin(ts16)

    ts16.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
