import numpy as np
import asyncio
import rospy
import cv2
import time
import rospy
import numpy as np

from lib.postion_estmation import ArUcoPosEstimator
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg  import Point32
from mavsdk             import System



class SensorHub:


    def __init__(self, drone, datahub):

        
        self.drone = drone # System()
                
        self.datahub = datahub # DataHub 

        self.bridge = CvBridge() # for receiving image topic


        rospy.Subscriber("/waypoints_input", Float32MultiArray, self.waypoints_updater)
        rospy.Subscriber("/bottom_cam/bottom_image_raw/compressed",CompressedImage,self.img_updater)

        #veranda_marker_detected in simulation
        rospy.Subscriber("marker_detected",Float32MultiArray,self.marker_detected_callback)

    #markrt detected 
    def marker_detected_callback(self, msg):
        
        self.datahub.cross_marker = msg.data
        self.datahub.cross_marker_detected = True
        self.datahub.nomarker_detected_trigger_time = time.time()
        
        print(self.datahub.nomarker_detected_trigger_time)


    def img_updater(self,img_msg):
        
        '''
        Callback function for downward image topic
        
        downward image ( Grayscale ) is used for
            - ArUco Marker detection & pose estimation
            - Visual Odometry
        '''


        try:
            # Convert your ROS Image message to OpenCV2
            img = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")

            self.datahub.img_bottom = img

        except CvBridgeError:

            print("CV Bridge Error")





    def waypoints_updater(self, msg):
        '''
        Callback function for Waypoint topic
        the waypoint topic has a data form of Float32Multiarray, which is 1d-array
        #ex
            wp#1 : [1,2,3]
            wp#2 : [2,3,4]
            v_mean : 3 m/s
            => serialized => msg.data = [3,1,2,3,2,3,4] : length = 1 + 3*n(n is a number of the waypoints)
            => deserialize => [[1,2],
                               [2,3],
                               [3,4]], v_mean = 3
        the frame of the waypoints is NED frame
        '''

        serialized = msg.data


        self.datahub.v_mean = serialized[0] #  pop v_mean
        serialized = serialized[1:]

        n = int(len(serialized)/3) # number of the waypoints

        wp = np.zeros((3,n)) # matrix whose columns are the waypoint vectors


        for i in range(n):  

            wp[:,i] = serialized[3*i:3*(i+1)]


        self.datahub.waypoints = wp # update the waypoint data in the datahub





    async def telem_posvel(self):
        '''
        Telemetry : position velocity ned
        '''

        while not self.datahub.is_connected:

            print("Telemetry : waiting for connection...",end="\r")

            await asyncio.sleep(0.01)

        async for pos_ned in self.drone.telemetry.position_velocity_ned():
            
            self.datahub.posvel_ned[0] = pos_ned.position.north_m
            self.datahub.posvel_ned[1] = pos_ned.position.east_m
            self.datahub.posvel_ned[2] = pos_ned.position.down_m
            self.datahub.posvel_ned[3] = pos_ned.velocity.north_m_s
            self.datahub.posvel_ned[4] = pos_ned.velocity.east_m_s
            self.datahub.posvel_ned[5] = pos_ned.velocity.down_m_s




    async def telem_posglobal(self):
        '''
        Telemetry : position Global
        '''

        while not self.datahub.is_connected:

            print("Telemetry : waiting for connection...",end="\r")

            await asyncio.sleep(0.01)

        async for pos_global in self.drone.telemetry.position():
            
            self.datahub.pos_global[0] = pos_global.latitude_deg
            self.datahub.pos_global[1] = pos_global.longitude_deg
            self.datahub.pos_global[2] = pos_global.absolute_altitude_m
            self.datahub.pos_global[3] = pos_global.relative_altitude_m




    async def telem_att_eular(self):
        '''
        Telemetry : attitude elular
        '''

        while not self.datahub.is_connected:

            # waiting for connection...
            
            await asyncio.sleep(0.01)

        # if connected
        async for att_eular in self.drone.telemetry.attitude_euler():

            self.datahub.attitude_eular[0] = np.deg2rad(att_eular.roll_deg)
            self.datahub.attitude_eular[1] = np.deg2rad(att_eular.pitch_deg)
            self.datahub.attitude_eular[2] = np.deg2rad(att_eular.yaw_deg)
            self.datahub.yaw_angle = att_eular.yaw_deg




    async def telem_att_quat(self):
        '''
        Telemetry : attitude elular
        '''

        while not self.datahub.is_connected:

            # waiting for connection...
            
            await asyncio.sleep(0.01)

        # if connected
        async for att_quat in self.drone.telemetry.attitude_quaternion():

            self.datahub.attitude_quat[0] = np.deg2rad(att_quat.w)
            self.datahub.attitude_quat[1] = np.deg2rad(att_quat.x)
            self.datahub.attitude_quat[2] = np.deg2rad(att_quat.y)
            self.datahub.attitude_quat[3] = np.deg2rad(att_quat.z)





    async def telem_home(self):
        '''
        Telemetry : global home position
        '''

        while not self.datahub.is_connected:

            # waiting for connection...
            
            await asyncio.sleep(0.01)

        # if connected
        async for pos in self.drone.telemetry.home():

            self.datahub.home[0] = pos.latitude_deg
            self.datahub.home[1] = pos.longitude_deg
            self.datahub.home[2] = pos.absolute_altitude_m
            self.datahub.home[3] = pos.relative_altitude_m
            




    async def telem_flightmode(self):
        '''
        Telemetry : flightmode str
        '''

        while not self.datahub.is_connected:
            
            # waiting for connection...

            await asyncio.sleep(0.01)

        # if connected
        async for mode in self.drone.telemetry.flight_mode():

            self.datahub.flight_mode = mode





    async def telem_armed(self):
        '''
        Telemetry : is armed bool
        '''

        while not self.datahub.is_connected:

            # waiting for connection...
            
            await asyncio.sleep(0.01)

        # if connected
        async for is_armed in self.drone.telemetry.armed():

            self.datahub.armed = is_armed





    async def telem_in_air(self):
        '''
        Telemetry : is in air bool
        '''

        while not self.datahub.is_connected:

            # waiting for connection...
            
            await asyncio.sleep(0.01)

        # if connected
        async for inair in self.drone.telemetry.in_air():

            self.datahub.is_in_air = inair
