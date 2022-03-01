#!/usr/bin/env python3
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/TIERS/ros-dwm1001-uwb-localization

    For more info on the documentation of DWM1001, go to the following links from Decawave: 
    1. https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
    2. https://www.decawave.com/product-documentation/    
"""

# Dependencies
import rclpy
from rclpy.node import Node
import numpy as np
import time, serial

from .dwm1001_apiCommands import DWM1001_API_COMMANDS
from .KalmanFilter import KalmanFilter as kf
from geometry_msgs.msg import PoseStamped
from .Helpers_KF import initConstVelocityKF 

from cg_msgs.msg import BeaconTag
from cg_msgs.msg import BeaconMultiTag


class dwm1001_localizer(Node):

    def __init__(self) :
        """
        Initialize the node, open serial port
        """        
        # Init node
        super().__init__('DWM1001_RTLS_Node')

        # allow serial port to be detected by user
        # NOTE: USB is assumed to be connected to ttyACM0. If not, need to modified it.
        # os.popen("sudo chmod 777 /dev/ttyACM0", "w")  
        
        self.multipleTags = BeaconMultiTag()
        self.pub_tags = self.create_publisher( BeaconMultiTag, "/dwm1001/multiTags", 10) 
        timer_period = 0.1 # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Empty dictionary to store topics being published
        self.topics = {}
        self.topics_kf = {}
        # Empty list for each tags of Kalman filter 
        self.kalman_list = [] 

        # Get port parameters
        self.declare_parameter('beacon_port', '/dev/ttyACM0')
        self.declare_parameter('beacon_verbose', False)

        # Serial port settings
        self.dwm_port = self.get_parameter('beacon_port').get_parameter_value().string_value
        self.verbose = self.get_parameter('beacon_verbose').get_parameter_value()
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            self.get_logger().info("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.get_logger().info("Reading DWM1001 coordinates and process them!")
        else:
            self.get_logger().info("Can't open port: "+ str(self.serialPortDWM1001.name))
    

    def timer_callback(self) :

        try:

            # just read everything from serial port
            serialReadLine = self.serialPortDWM1001.read_until()

            try:
                # Publish the Raw Pose Data directly from the USB                     
                self.publishTagPositions(serialReadLine)    

                ############### Kalman Filter ###############
                # Use Kalman filter to process the data and publish it 
                serDataList = [x.strip() for x in serialReadLine.strip().split(b',')]

                # If getting a tag position
                if b"POS" in serDataList[0] :
                    self.get_logger().info(serDataList[0])
                    

                    tag_id = int(serDataList[1])  
                    # tag_id = str(serDataList[1], 'UTF8')  # IDs in 0 - 15
                    tag_macID = str(serDataList[2], 'UTF8')
                    t_pose_x = float(serDataList[3])
                    t_pose_y = float(serDataList[4])
                    t_pose_z = float(serDataList[5])   

                    # To use this raw pose of DWM1001 as a measurement data in KF
                    t_pose_list = [t_pose_x, t_pose_y, t_pose_z]

                    # Discard the pose data from USB if there exists "nan" in the values
                    if(np.isnan(t_pose_list).any()):
                        self.get_logger().info("Serial data include Nan!")
                        return
                    else:
                        t_pose_xyz = np.array(t_pose_list) # numpy array

                    # t_pose_xyz = np.array([t_pose_x, t_pose_y, t_pose_z])
                    t_pose_xyz.shape = (len(t_pose_xyz), 1)    # force to be a column vector                                       

                    if tag_macID not in self.kalman_list:   # TODO: tag_macID
                        # self.kalman_list.append(tag_id)
                        self.kalman_list.append(tag_macID)
                        # Suppose constant velocity motion model is used (x,y,z and velocities in 3D)
                        A = np.zeros((6,6))
                        H = np.zeros((3, 6))  # measurement (x,y,z without velocities) 

                        # For constant acceleration model, define the place holders as follows:
                        # A = np.zeros((9,9)) 
                        # H = np.zeros((3, 9)) 
                        # idx = self.kalman_list.index(tag_id)
                        self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # self.kalman_list[tag_id] = kf(A, H, tag_macID) # create KF object for tag id
                        # print(self.kalman_list[tag_id].isKalmanInitialized)
                    
                    # idx_kf = self.kalman_list.index(tag_id)
                    # idx = self.kalman_list.index(tag_macID)  # index of the Tag ID

                    if self.kalman_list[tag_id].isKalmanInitialized == False:  
                        # Initialize the Kalman by asigning required parameters
                        # This should be done only once for each tags
                        A, B, H, Q, R, P_0, x_0  = initConstVelocityKF() # for const velocity model
                        # A, B, H, Q, R, P_0, x_0  = initConstAccelerationKF() # for const acceleration model
                        
                        self.kalman_list[tag_id].assignSystemParameters(A, B, H, Q, R, P_0, x_0)  # [tag_id]
                        self.kalman_list[tag_id].isKalmanInitialized = True                            
                        # print(self.kalman_list[tag_id].isKalmanInitialized)                           
                
                    self.kalman_list[tag_id].performKalmanFilter(t_pose_xyz, 0)  
                    t_pose_vel_kf = self.kalman_list[tag_id].x_m  # state vector contains both pose and velocities data
                    t_pose_kf = t_pose_vel_kf[0:3]  # extract only position data (x,y,z)
                    # print(t_pose_kf)                      
                    self.publishTagPoseKF(tag_id, tag_macID, t_pose_kf)
                    # print(len(self.kalman_list))
                    
                ############### Kalman Filter ###############

            except IndexError:
                self.get_logger().info("Found index error in the network array!DO SOMETHING!")

        except KeyboardInterrupt:
            self.get_logger().info("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
             self.get_logger().info("Quitting, and sending reset command to dev board")
             # self.serialPortDWM1001.reset_input_buffer()
             # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
             # self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

             # serialReadLine = self.serialPortDWM1001.read_until()
             # if b"reset" in serialReadLine:
             #     self.get_logger().info("succesfully closed ")
             #     self.serialPortDWM1001.close()


    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """ 
        ser_pose_data = [x.strip() for x in serialData.strip().split(b',')]

        # If getting a tag position
        if b"POS" in ser_pose_data[0] :
            #rospy.loginfo(arrayData)  # just for debug

            tag_id = str(ser_pose_data[1], 'UTF8')  # IDs in 0 - 15
            tag_macID = str(ser_pose_data[2], 'UTF8')

            ps = PoseStamped()
            ps.pose.position.x = float(ser_pose_data[3])
            ps.pose.position.y = float(ser_pose_data[4])
            ps.pose.position.z = float(ser_pose_data[5])
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            ps.header.stamp = self.get_clock().now().to_msg()   
            ps.header.frame_id = tag_macID # TODO: Currently, MAC ID of the Tag is set as a frame ID 

            raw_pose_xzy = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]

            # TODO: PoseStamped() may be replaced with compatible Custom msgs for uniform msg type
            # Assign the PoseStamped msg into CustomTag msg
            tag = BeaconTag()
            tag.header = ps.header
            tag.pose_x = ps.pose.position.x
            tag.pose_y = ps.pose.position.y
            tag.pose_z = ps.pose.position.z
            tag.orientation_x = ps.pose.orientation.x
            tag.orientation_y = ps.pose.orientation.y
            tag.orientation_z = ps.pose.orientation.z
            tag.orientation_z = ps.pose.orientation.w

            if tag_id not in self.topics:
                self.topics[tag_id] = self.create_publisher( PoseStamped, "/dwm1001/id_" + tag_macID + "/pose", 10) 
                self.multipleTags.TagsList.append(tag) # append custom Tags into the multiple tag msgs
                
                #rospy.loginfo("New tag {}. x: {}m, y: {}m, z: {}m".format(
                #    str(tag_id),
                #    ps.pose.position.x,
                #    ps.pose.position.y,
                #    ps.pose.position.z
                #))
            
            # self.topics[tag_id].publish(ps)
            
            # Publish only pose data without "NAN"
            if(np.isnan(raw_pose_xzy).any()): 
                pass
            else:
                self.topics[tag_id].publish(ps) 
                self.multipleTags.TagsList[int(tag_id)]= tag

                # Publish multiple tags data for RVIZ visualization 
                # pub_tags = rospy.Publisher("/dwm1001/multiTags", MultiTags, queue_size=100)                  
             
            self.pub_tags.publish(self.multipleTags)    
                        

            # if self.verbose :
            #     rospy.loginfo("Tag " + str(tag_macID) + ": "
            #         + " x: "
            #         + str(ps.pose.position.x)
            #         + " y: "
            #         + str(ps.pose.position.y)
            #         + " z: "
            #         + str(ps.pose.position.z)
            #     )
    
    # Publish Tag positions using KF 
    def publishTagPoseKF(self, id_int, id_str, kfPoseData):

        ps = PoseStamped()
        ps.pose.position.x = float(kfPoseData[0])
        ps.pose.position.y = float(kfPoseData[1])
        ps.pose.position.z = float(kfPoseData[2])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = self.get_clock().now().to_msg()   
        ps.header.frame_id = id_str # use MAC ID of the Tag as a frame ID for ROS

        if id_int not in self.topics_kf:
            self.topics_kf[id_int] = self.create_publisher( PoseStamped, "/dwm1001/id_" + str(id_str) + "/pose_kf", 10) 

        self.topics_kf[id_int].publish(ps)
            

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
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