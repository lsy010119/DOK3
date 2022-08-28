import numpy as np
import asyncio
import mavsdk
import time 
import threading
import rospy


from mavsdk             import System
from data_hub           import DataHub
from connector			import Connector
from sensor_hub         import SensorHub
from lib.lidar_processor import LiDARProcessor 
from lib.jps            import JPS
from std_msgs.msg       import Float32MultiArray
from sensor_msgs.msg    import PointCloud
from geometry_msgs.msg  import Point32
from nav_msgs.msg       import Odometry






class FCReader(threading.Thread):
	'''
	FC Reader
		Connector
		- MAVLINK connection by MAVSDK API server
		Sensorhub
		
		- reads telemetry data
		- subscribes waypoint topic, img topic
	'''
	def __init__(self, drone, datahub):
		super().__init__()
		self.drone = drone
		self.datahub = datahub

		self.connector = Connector(self.drone,self.datahub)
		self.sensorhub = SensorHub(self.drone,self.datahub)

	
	async def connect(self):
		await self.connector.connect() # connect to PX4


	def run(self):

		print("===== FCReader start =====")

		# Create new Event loop for FC Reader Thread
		telem_event_loop = asyncio.new_event_loop()
		asyncio.set_event_loop(telem_event_loop)
		
		# run connect function & telemetry reader tasks
		telem_event_loop.run_until_complete(\
			asyncio.gather( self.connect(),\
							self.sensorhub.telem_posvel(),\
							self.sensorhub.telem_posglobal(),\
							self.sensorhub.telem_att_eular(),\
							self.sensorhub.telem_att_quat(),\
							self.sensorhub.telem_home(),\
							self.sensorhub.telem_flightmode(),\
							self.sensorhub.telem_armed(),\
							self.sensorhub.telem_in_air()))





class Visualizer(threading.Thread):


	def __init__(self, datahub):
		super().__init__()
		self.datahub = datahub
		
		self.mapper = LiDARProcessor(self.datahub)

		self.trajec = PointCloud()
		self.trajec.header.frame_id = "velodyne"       
		self.trajec_pub = rospy.Publisher("/traj",PointCloud,queue_size=1)

		self.wp_viz = PointCloud()
		self.wp_viz.header.frame_id = "velodyne"       
		self.wp_viz_pub = rospy.Publisher("/waypoints",PointCloud,queue_size=1)

		self.wp_jps_viz = PointCloud()
		self.wp_jps_viz.header.frame_id = "velodyne"       
		self.wp_jps_viz_pub = rospy.Publisher("/jps_wp",PointCloud,queue_size=1)

		self.pose_viz = Odometry()
		self.pose_viz.header.frame_id = "velodyne"
		self.pose_viz_pub = rospy.Publisher("/pose", Odometry, queue_size=1)


		self.jps_map_pub = rospy.Publisher("/map",Float32MultiArray,queue_size=1)
		self.jps_map_norotated_pub = rospy.Publisher("/map_no_rotated",Float32MultiArray,queue_size=1)
	
		self.posatt_pub = rospy.Publisher("/posned_atteul",Float32MultiArray,queue_size=1)

		self.waypoints_pub = rospy.Publisher("/waypoints", Float32MultiArray, queue_size=1)



	
	def gridizer(self, map, x_des):

        # map = self.mapper.voxelize()

        # start point is the center of the map
		start = np.array([int(len(map)//2),int(len(map)//2)])

        # goal point input is a relative position of destination
        # goal_input = x_des[:2] - self.ned2xyz(self.datahub.posvel_ned[:3])[:2]
		goal_input = x_des[:2] - self.datahub.posvel_ned[:2]

        # Transform the goal point into row-col coordinate system for JPS
		goal = np.array([-int(goal_input[0]/self.mapper.grid_size),\
                            int(goal_input[1]/self.mapper.grid_size)]) + start

		jps = JPS(map,start,goal,self.datahub.posvel_ned[3:5])


		wp = jps.run()

		nodes = np.zeros(np.shape(jps.map))


        ##### Visualize ######
		try:
			for i in range(len(wp[0])):

				nodes[start[0]-wp[0,i],wp[1,i]+start[0]]=2

		except:
			pass
        
		final_map = jps.map + nodes
		final_map[0,0] = 2
		final_map[start[0],start[1]] = 3

		final_map = final_map.astype(float)


		velocity = self.datahub.posvel_ned[3:5].astype(float)

		final_map = np.hstack((final_map.flatten(),velocity))

		self.datahub.jps_map = final_map # for visualizer

        ######################

		if len(wp) != 0: # if there are obstacles on my way

            # JPS is runned in 2D..
            # z axis must be appended
			wp = wp * self.mapper.grid_size

            # convert the relative position of wp into absolute position 
            # wp[0] = wp[0] + self.ned2xyz(self.datahub.posvel_ned[:3])[0] 
            # wp[1] = wp[1] + self.ned2xyz(self.datahub.posvel_ned[:3])[1]
			wp[0] = wp[0] + self.datahub.posvel_ned[0] 
			wp[1] = wp[1] + self.datahub.posvel_ned[1]

            # append z axiz elements to waypoints
			wp = np.vstack( (wp, (self.datahub.posvel_ned[2])*np.ones( (1,len(wp[0]) ) ) ))

			wp = (wp.flatten()).tolist()

		wp_msg = Float32MultiArray()
		wp_msg.data = wp
		self.waypoints_pub.publish(wp_msg)




	def gridizer_no_rotated(self, map, x_des):

        # map = self.mapper.voxelize()

        # start point is the center of the map
		start = np.array([int(len(map)//2),int(len(map)//2)])

        # goal point input is a relative position of destination
        # goal_input = x_des[:2] - self.ned2xyz(self.datahub.posvel_ned[:3])[:2]
		goal_input = x_des[:2] - self.datahub.posvel_ned[:2]

        # Transform the goal point into row-col coordinate system for JPS
		goal = np.array([-int(goal_input[0]/self.mapper.grid_size),\
                            int(goal_input[1]/self.mapper.grid_size)]) + start

		jps = JPS(map,start,goal,self.datahub.posvel_ned[3:5])


		wp = jps.run()

		nodes = np.zeros(np.shape(jps.map))


        ##### Visualize ######
		try:
			for i in range(len(wp[0])):

				nodes[start[0]-wp[0,i],wp[1,i]+start[0]]=2

		except:
			pass
        
		final_map = jps.map + nodes
		final_map[0,0] = 2
		final_map[start[0],start[1]] = 3

		final_map = final_map.astype(float)


		velocity = self.datahub.posvel_ned[3:5].astype(float)

		final_map = np.hstack((final_map.flatten(),velocity))

		self.datahub.jps_map_no_rotated = final_map # for visualizer

        ######################

		if len(wp) != 0: # if there are obstacles on my way

            # JPS is runned in 2D..
            # z axis must be appended
			wp = wp * self.mapper.grid_size

            # convert the relative position of wp into absolute position 
            # wp[0] = wp[0] + self.ned2xyz(self.datahub.posvel_ned[:3])[0] 
            # wp[1] = wp[1] + self.ned2xyz(self.datahub.posvel_ned[:3])[1]
			wp[0] = wp[0] + self.datahub.posvel_ned[0] 
			wp[1] = wp[1] + self.datahub.posvel_ned[1]

            # append z axiz elements to waypoints
			wp = np.vstack( (wp, (self.datahub.posvel_ned[2])*np.ones( (1,len(wp[0]) ) ) ))

			wp = (wp.flatten()).tolist()




	def run(self):

		while not rospy.is_shutdown():
			
			self.pose_viz.pose.pose.position.x =  self.datahub.posvel_ned[1]
			self.pose_viz.pose.pose.position.y =  self.datahub.posvel_ned[0]
			self.pose_viz.pose.pose.position.z = -self.datahub.posvel_ned[2]

			self.pose_viz.pose.pose.orientation.w = self.datahub.attitude_quat[0]
			self.pose_viz.pose.pose.orientation.x = self.datahub.attitude_quat[1]
			self.pose_viz.pose.pose.orientation.y = self.datahub.attitude_quat[2]
			self.pose_viz.pose.pose.orientation.z = self.datahub.attitude_quat[3]


			posatt_msg = Float32MultiArray()
			posatt_msg.data = ( np.hstack((self.datahub.posvel_ned,self.datahub.attitude_eular)) ).tolist()



			self.posatt_pub.publish(posatt_msg)

			rostime = rospy.Time.now()
			self.pose_viz.header.stamp = rostime
			self.pose_viz_pub.publish(self.pose_viz)



			if len(self.datahub.jps_map) != 0:

				map_msg = Float32MultiArray()

				map_msg.data = self.datahub.jps_map.tolist()

				self.jps_map_pub.publish(map_msg)


			if len(self.datahub.jps_map_no_rotated) != 0:

				map_msg = Float32MultiArray()

				map_msg.data = self.datahub.jps_map_no_rotated.tolist()

				self.jps_map_norotated_pub.publish(map_msg)


			map_rotated = self.mapper.generate_grid()
			map_not_rotated = self.mapper.generate_grid_no_rotated()

			self.gridizer(map_rotated,np.array([30,0,0]))
			self.gridizer_no_rotated(map_not_rotated,np.array([30,0,0]))


			time.sleep(0.01)




class Master:

	def __init__(self, delt, traj_update_period, grid_size, threshold, max_range, expension_size, bottom_cam_mtx, bottom_dist_coeff, ip, port, visualize=True, communication=True):
		
		rospy.init_node("dok3")

		self.datahub = DataHub(delt, traj_update_period, grid_size, threshold, max_range, expension_size, bottom_cam_mtx, bottom_dist_coeff)	

		self.drone_I = System()
		self.drone_O = System()

		self.fc_reader = FCReader(self.drone_O, self.datahub)
		self.visualize = Visualizer(self.datahub)

		self.fc_reader.daemon = True
		self.visualize.daemon = True
				
		self.fc_reader.start()
		self.visualize.start()



	def run(self):	
		
		while not rospy.is_shutdown():

			# print("posvelned",self.datahub.posvel_ned)
			# print("posatt",np.rad2deg(self.datahub.attitude_eular))
			# print("posvelned",self.datahub.posvel_ned)
			# print("posvelned",self.datahub.posvel_ned)
			# print("posvelned",self.datahub.posvel_ned)
			# print("posvelned",self.datahub.posvel_ned)
			time.sleep(0.1)




if __name__ == "__main__":

	# Controller

	delt = 0.1 				# Control Time Interval for dicrete-time dynamic system 

	traj_update_period = 0.5  # Period of updating trajectory 

	# LiDAR Processor

	grid_size = 0.5		# voxel size [m]

	threshold = 1			# threshold for voxelization

	max_range = 20			# the maximum range of LiDAR 

	expension_size = 6
	
	# Image Procesor

	bottom_cam_mtx = [[347.344668, 0.00000000, 317.843671],
                	  [0.00000000, 346.900900, 255.698665],
                	  [0.00000000, 0.00000000, 1.00000000]]

	bottom_dist_coeff = np.array([[ -0.279997, 0.058631, 0.002795, -0.000103, 0.000000]])



	server_ip = '165.246.139.32'
	server_port = 9502

	master = Master(delt, traj_update_period,\
                    grid_size, threshold, max_range, expension_size,\
                    bottom_cam_mtx, bottom_dist_coeff,\
					server_ip,server_port,\
					visualize=True,
					communication=False)        

	master.run()

