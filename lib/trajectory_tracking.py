from threading import local
import numpy as np
import copy
import asyncio
import time

import rospy
from lib.trajectory_generator import TrajectoryGenerator
from lib.jps                  import JPS

from std_msgs.msg       import Float32MultiArray
from sensor_msgs.msg    import PointCloud
from geometry_msgs.msg  import Point32
from nav_msgs.msg       import Odometry
from mavsdk.offboard    import VelocityNedYaw,\
                               PositionNedYaw,\
                               VelocityBodyYawspeed

import matplotlib.pyplot as plt     
from matplotlib import colors 
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes

class TrajectoryTracker:


    def __init__(self, drone, datahub, lidarprocessor):

        self.drone = drone 
        self.datahub = datahub

        self.delt = datahub.delt
        self.period = datahub.traj_update_period

        self.generator = TrajectoryGenerator(self.delt)
        self.mapper = lidarprocessor






    def local_planner(self, x_des):

        if self.datahub.SITL:
            map = self.mapper.generate_grid_sim()
        else:
            map = self.mapper.generate_grid_real()

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

        return wp






        
    async def trajectory_tracking(self, x_des, wp, v_mean): # input : NED frame


        wp_passed = 0

        self.datahub.heading_wp += 1

        vel_traj_list = []
        vel_actual_list = []


        n_update = int(self.period/self.delt)       # timesteps for update period

        traj_log = np.zeros((3,1))

        while True:

            start_avoidance = time.time()

            if len(wp) != 0 and np.shape(wp)[1] != 0:
                
                avoidance_wp = self.local_planner(wp[:,0])    # find avoidance path via JPS in local planner

                
                try:
                    augmented_wp = np.hstack((avoidance_wp,wp))
                    n_avoid = len(avoidance_wp[0])
        
                except:
                
                    augmented_wp = wp
                    n_avoid = 0


            else:
                avoidance_wp = self.local_planner(x_des)    # find avoidance path via JPS in local planner

                augmented_wp = avoidance_wp


            x_0 = self.datahub.posvel_ned 
            # initial state 
            # position : current position
            # velocity : mean of last velocity command and current velocity   

            # x_0[3:] = 0.9*last_vel_command + 0.1*x_0[3:].copy()

            traj,tk = self.generator.generate(x_0,x_des,augmented_wp,v_mean,self.period)

            cur_yaw = np.rad2deg(self.datahub.attitude_eular[2])

            self.datahub.traj = traj # for visualizer



            ########################### orientation ###########################

            try:

                if len(wp) != 0:

                    delta_n = wp[0,0] - self.datahub.posvel_ned[0]
                    delta_e = wp[1,0] - self.datahub.posvel_ned[1]

                    if np.linalg.norm(wp[:2,0]-self.datahub.posvel_ned[:2]) < 3:

                        target_yaw = cur_yaw

                    else:

                        target_yaw = np.rad2deg(np.arctan2( delta_e, delta_n ))

                else:

                    delta_n = x_des[0] - self.datahub.posvel_ned[0]
                    delta_e = x_des[1] - self.datahub.posvel_ned[1]

                    if np.linalg.norm(x_des[:2]-self.datahub.posvel_ned[:2]) < 3:

                        target_yaw = cur_yaw

                    else:

                        target_yaw = np.rad2deg(np.arctan2( delta_e, delta_n ))

            except:

                target_yaw = cur_yaw


            ### yaw rotation direction ###

            if abs( cur_yaw - target_yaw ) <= 180:

                delta_yaw = target_yaw - cur_yaw

            else:

                if ( cur_yaw - target_yaw ) >= 0: 
                    delta_yaw = 360 + target_yaw - cur_yaw
                else: 
                    delta_yaw = -360 + target_yaw - cur_yaw


            ###################################################################


            if len(traj[0]) > n_update: # if the path spends over update period


                for i in range(n_update):


                    if len(wp) != 0:


                        if np.shape(wp)[1] == 0:

                            wp = np.array([])
                        

                        elif i >= tk[0]:

                            self.datahub.heading_wp += 1

                            print(f"waypoint #{wp_passed+1}  passed")
                            
                            self.datahub.heading_wp += 1
                            
                            wp = np.delete(wp,0,axis=1) # discard the waypoint passed through

                            break


                    vel = traj[3:,0]
                    pos = traj[:3,0]

                    traj = traj[:,1:]

                    traj_log = np.hstack((traj_log, np.reshape(self.datahub.posvel_ned[:3],(3,1)) ))

                    yaw_con = cur_yaw + i * delta_yaw/n_update

                    await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(vel[0], vel[1], vel[2], yaw_con ))

                    vel_traj_list.append(np.linalg.norm(vel))
                    vel_actual_list.append(np.linalg.norm(self.datahub.posvel_ned[3:]))

                    await asyncio.sleep(self.datahub.delt)   
                
                # print(end_tracking-start_tracking)



            else:


                for i in range(len(traj[0])):


                    vel = traj[3:,0]
                    pos = traj[:3,0]

                    yaw_con = cur_yaw + i*delta_yaw/len(traj[0])

                    traj_log = np.hstack((traj_log, np.reshape(self.datahub.posvel_ned[:3],(3,1)) ))
                    

                    traj = traj[:,1:]


                    await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(vel[0], vel[1], vel[2], yaw_con ))

                    vel_traj_list.append(np.linalg.norm(vel))
                    vel_actual_list.append(np.linalg.norm(self.datahub.posvel_ned[3:]))
                    await asyncio.sleep(self.datahub.delt)     

                break
        
        # print(self.datahub.attitude_eular[2])
        # plt.plot(range(len(vel_traj_list)),vel_traj_list,label="calculated")
        # plt.plot(range(len(vel_actual_list)),vel_actual_list,label="actual")
        # plt.legend()
        # plt.show()