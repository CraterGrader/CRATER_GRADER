#!/usr/bin/env python3
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/cliansang/uwb-tracking-ros

    For more info on the documentation of DWM1001, go to the following links from Decawave: 
    1. https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
    2. https://www.decawave.com/product-documentation/    
"""

# Dependencies
import rclpy
from rclpy.node import Node
import numpy as np
import time, serial, yaml, os
from .dwm1001_apiCommands import DWM1001_API_COMMANDS
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from ament_index_python.packages import get_package_share_directory


class dwm1001_localizer(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Initialize node
        super().__init__('uwb_beacon_rtls')

        uwb_tag_data_path =  os.path.join(
            get_package_share_directory('uwb_beacon_rtls'),
            'config',
            'tag_transforms.yaml'
            )

        with open(uwb_tag_data_path, 'r') as stream:
            tag_params = yaml.load(
            stream,
            Loader=yaml.SafeLoader
            )
        
        self.declare_parameter('beacon_hz', 10)
        self.beacon_hz = self.get_parameter('beacon_hz').get_parameter_value().double_value
        self.timer = self.create_timer(self.beacon_hz, self.timer_callback)
        
        self.declare_parameter('beacon_verbose', False)
        self.beacon_verbose = self.get_parameter('beacon_verbose').get_parameter_value().bool_value

        self.declare_parameter('publish_raw_tags', False)
        self.publish_raw_tags = self.get_parameter('publish_raw_tags').get_parameter_value().bool_value

        self.tag_ids = tag_params['tag_ids']

        self.tag_names = tag_params['tag_names']

        tag_transform_str = tag_params['tag_transforms']
        self.tag_transforms = {}
        for i, tid in enumerate(self.tag_ids):
            self.tag_transforms[tid] = tag_transform_str[i]

        self.declare_parameter('positional_rms', 0.0)
        self.positional_rms = self.get_parameter('positional_rms').get_parameter_value().double_value

        self.declare_parameter('raw_topic', 'raw_pose')
        self.raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value
        if self.publish_raw_tags:
            self.raw_publishers = {}
            for tag_id in self.tag_ids:
                self.raw_publishers[tag_id] = self.create_publisher( PoseWithCovarianceStamped, self.raw_topic + "tag_" + str(self.tag_names[tag_id]), 10) 

        self.declare_parameter('beacon_port', 'ttyACM0')
        self.dwm_port = self.get_parameter('beacon_port').get_parameter_value().string_value
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        # Close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        time.sleep(1)
        self.serialPortDWM1001.open()

        if(self.serialPortDWM1001.isOpen()):
            self.get_logger().info("Port opened: "+ str(self.serialPortDWM1001.name) )
            self.initializeDWM1001API()
            # Leave time to DWM1001 to wake up
            time.sleep(2)
            # Send command lec, to get positions in CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.get_logger().info("Reading DWM1001 coordinates")
        else:
            self.get_logger().info("Can't open port: "+ str(self.serialPortDWM1001.name))
    

    def timer_callback(self):

        detected_poses = {}
        for n in range(len(self.tag_ids)):

            serialReadLine = self.serialPortDWM1001.read_until()

            serDataList = [x.strip() for x in serialReadLine.strip().split(b',')]
            # self.get_logger().info(f"Raw Data: {serDataList}")

            # If valid tag position
            if b"POS" in serDataList[0]:
                
                try:
                    tag_id = serDataList[2].decode()  # IDs in 0 - 15
                    t_pose_x = float(serDataList[3])
                    t_pose_y = float(serDataList[4])
                    t_pose_z = float(serDataList[5])   
                except:
                    if self.beacon_verbose:
                        self.get_logger().info("Failed to parse tag data: ")
                    continue
                
                if self.beacon_verbose and tag_id not in self.tag_ids:
                    self.get_logger().info(f"Got an unknown tag_id: {tag_id}")
                    continue

                if tag_id in detected_poses:
                    continue

                # To use this raw pose of DWM1001 as a measurement data
                t_pose_xyz =  np.array([t_pose_x, t_pose_y, t_pose_z])

                # Discard the pose data from USB if there exists "nan" in the values
                if(np.isnan(t_pose_xyz).any()):
                    if self.beacon_verbose:
                        self.get_logger().info("Serial data include Nan!")
                    continue

                if self.publish_raw_tags:
                    self.publishTagPose(t_pose_xyz - np.array(self.tag_transforms[tag_id]), "map", self.raw_publishers[tag_id])    

                assert t_pose_xyz.shape == np.array(self.tag_transforms[tag_id]).shape
                detected_poses[tag_id] = t_pose_xyz - np.array(self.tag_transforms[tag_id])
        
        if self.beacon_verbose:
            self.get_logger().info(f"Got tags: {list(detected_poses.keys())}")

        if len(detected_poses) == 0:
            if self.beacon_verbose:
                self.get_logger().info(f"No pose detections found")
            return


    def publishTagPose(self, pos, frame_id, publisher):
        """
        Publish Position using provided publisher
        :param: [Double] PoseData, String frame_id
        :returns: none
        """

        ps = PoseWithCovarianceStamped()
        ps.pose.pose.position.x = float(pos[0])
        ps.pose.pose.position.y = float(pos[1])
        ps.pose.pose.position.z = float(pos[2])
        ps.pose.pose.orientation.x = 0.0
        ps.pose.pose.orientation.y = 0.0
        ps.pose.pose.orientation.z = 0.0
        ps.pose.pose.orientation.w = 1.0
        ps.header.stamp = self.get_clock().now().to_msg()   
        ps.header.frame_id = frame_id

        #Assuming diagonal covariance matrix
        #Setting rotation covariance to 1 by default, but should be thrown out during sensor fusion
        ps.pose.covariance = [self.positional_rms,0.0,0.0,0.0,0.0,0.0,
                                0.0,self.positional_rms,0.0,0.0,0.0,0.0,
                                0.0,0.0,self.positional_rms,0.0,0.0,0.0,
                                0.0,0.0,0.0,0.0,0.0,0.0,
                                0.0,0.0,0.0,0.0,0.0,0.0,
                                0.0,0.0,0.0,0.0,0.0,0.0]
        publisher.publish(ps)
            

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """

        #Initialization procedure
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)


def main(args=None):

    rclpy.init(args=args)

    dwm1001 = dwm1001_localizer()
    dwm1001.initializeDWM1001API()      

    rclpy.spin(dwm1001)

    dwm1001.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()