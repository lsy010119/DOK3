import numpy as np
import copy
import asyncio
import time

import rospy
from lib.trajectory_generator import TrajectoryGenerator
from lib.search_planner       import SearchPlanner

from mavsdk.offboard    import VelocityNedYaw,\
                               PositionNedYaw,\
                               VelocityBodyYawspeed

import matplotlib.pyplot as plt     
from matplotlib import colors 
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes


class Search:


    def __init__(self, drone, datahub, lidarprocessor):

        self.drone = drone 
        self.datahub = datahub

        self.delt = datahub.delt
        self.period = datahub.traj_update_period

        self.generator = TrajectoryGenerator(self.delt)
        self.mapper = lidarprocessor



        
    async def run(self): # input : NED frame

        n_update = int(3/self.delt) 
        
        marker_detected = False
        
        while not marker_detected:

            map_body,veranda_pos = self.mapper.voxelize_veranda(6)


            if len(veranda_pos) == 0:

                print("theres no fucking veranda anywhere")

            else:

                self.search_planner = SearchPlanner(map_body)

                search_point_body = self.search_planner.run(CW=True) * self.datahub.voxel_size
                

                ##### Visualize ######

                try:
                
                    map_body[int(len(map_body)//2)-search_point_body[0], int(len(map_body)//2)+search_point_body[1]]

                except:

                    pass
                
                map_body[0,0] = 2

                map_body[int(len(map_body)//2),int(len(map_body)//2)] = 3

                map_body = map_body.astype(float)

                velocity = self.datahub.posvel_ned[3:5].astype(float)

                map_body = np.hstack((map_body.flatten(),velocity))

                self.datahub.jps_map = map_body # for visualizer

                ######################


                try:
                    search_point_ned = self.mapper.transform_mtrx(np.zeros(3),self.datahub.attitude_eular)@search_point_body
                    veranda_pos = self.mapper.transform_mtrx(np.zeros(3),self.datahub.attitude_eular)@veranda_pos
                                
                    print(veranda_pos)

                    # print(search_point_ned)

                    x_des = np.append(search_point_ned[:3],np.zeros(3))

                    x_des[2] = self.datahub.posvel_ned[2]

                    x_0 = self.datahub.posvel_ned # initial state = current state

                    traj,_ = self.generator.generate(x_0,x_des,np.array([]),1)

                    self.datahub.traj = traj # for visualizer

                except:
                    traj = np.array([[],[],[]])
                    self.datahub.state = "Hold"

                    self.datahub.action = "hold"


                if len(traj[0]) > n_update:

                    for i in range(n_update):


                        vel = traj[3:,0]

                        traj = traj[:,1:]

                        ## orientation ##

                        yaw = self.datahub.attitude_eular[2]
                    
                        delta_n = veranda_pos[0] - self.datahub.posvel_ned[0]
                        delta_e = veranda_pos[1] - self.datahub.posvel_ned[1]

                        yaw = np.rad2deg(np.arctan2( delta_e, delta_n ))

                        # print(yaw)

                        ######

                        await self.drone.offboard.set_velocity_ned(
                                VelocityNedYaw(vel[0], vel[1], vel[2], yaw))


                        await asyncio.sleep(self.datahub.delt)

                else:

                    for i in range(len(traj[0])):


                        vel = traj[3:,0]

                        traj = traj[:,1:]

                        ## orientation ##

                        yaw = self.datahub.attitude_eular[2]
                    
                        delta_n = veranda_pos[0] - self.datahub.posvel_ned[0]
                        delta_e = veranda_pos[1] - self.datahub.posvel_ned[1]

                        yaw = np.rad2deg(np.arctan2( delta_e, delta_n ))

                        # print(yaw)

                        ######

                        await self.drone.offboard.set_velocity_ned(
                                VelocityNedYaw(vel[0], vel[1], vel[2], yaw))


                        await asyncio.sleep(self.datahub.delt)
