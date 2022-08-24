#!/usr/bin/python3
import numpy as np
import rospy
import time
import asyncio
import threading

from sensor_msgs.msg    import PointCloud
from sensor_msgs.msg    import LaserScan
from geometry_msgs.msg  import Point32


class LiDARProcessor:


    def __init__(self, datahub):

        self.datahub = datahub

        self.range_data = None
        self.del_theta = None

        self.max_range = datahub.max_range
        self.grid_size = datahub.grid_size
        self.threshold = datahub.threshold
        self.expension_size = datahub.expension_size

        self.map = np.zeros((int(2*self.max_range/self.grid_size + 1),int(2*self.max_range/self.grid_size + 1)))

        self.map[:,:2] = np.ones((len(self.map),2))
        self.map[:,-2:]= np.ones((len(self.map),2))
        self.map[:2,:] = np.ones((2,len(self.map[0])))
        self.map[-2:,:]= np.ones((2,len(self.map[0])))
        
        #Laserscan 저거 시뮬에서 
        # rospy.Subscriber("/dok3/lidar",LaserScan,self.callback, queue_size=1)
        # rospy.Subscriber("/target_points",PointCloud, self.lidar_callback, queue_size=1)
        rospy.Subscriber("/processed_cloud",PointCloud, self.lidar_callback, queue_size=1)




    def lidar_callback(self, msg):

        target_points = []
        
        for point in msg.points:
        
            target_points.append(point)

        self.datahub.target_points = target_points




    def callback(self,lidar_data):

        self.range_data = lidar_data.ranges
        self.del_theta = lidar_data.angle_increment




    def transform_mtrx(self, position_ned, att_eular):


        pos_n = position_ned[0]
        pos_e = position_ned[1]
        pos_d = position_ned[2]

        u = att_eular[0] # roll
        v = att_eular[1] # pitch
        w = att_eular[2] # yaw
    
        mtx = np.array([[ np.cos(v)*np.cos(w),      np.sin(u)*np.sin(v)*np.cos(w)-np.cos(u)*np.sin(w),      np.sin(u)*np.sin(w)+np.cos(u)*np.sin(v)*np.cos(w) , pos_n],
                        [ np.cos(v)*np.sin(w),      np.cos(u)*np.cos(w)+np.sin(u)*np.sin(v)*np.sin(w),      np.cos(u)*np.sin(v)*np.sin(w)-np.sin(u)*np.cos(w) , pos_e],
                        [ - np.sin(v)        ,      np.sin(u)*np.cos(v)                              ,      np.cos(u)*np.cos(v)                               , pos_d],
                        [ 0                  ,      0                                                ,      0                                                 , 1    ]])

        return mtx




    def check_wall(self): # approach #1

        position = np.zeros(3) # ned

        cnt = 0

        for n, ray in enumerate(self.range_data):

            # ned position in body frame
            if 0.5< ray < self.max_range:

                position[0] += - np.cos(n*self.del_theta)            
                position[1] += np.sin(n*self.del_theta)           
                position[2] += 0            

                cnt += 1

        position /= cnt

        if cnt == 0:

            return -1

        else:

            return position




    def voxelize_veranda(self, veranda_expension_size): # approach #2


        self.map = np.zeros((np.shape(self.map)))
        self.map[:,:2] = np.ones((len(self.map),2))
        self.map[:,-2:]= np.ones((len(self.map),2))
        self.map[:2,:] = np.ones((2,len(self.map[0])))
        self.map[-2:,:]= np.ones((2,len(self.map[0])))

        position = np.zeros(4) # ned

        cnt = 0

        if self.range_data == None:
            print("not yet")

        else:

            points = np.zeros((4, len(self.range_data)))

            downsampled = {}

            for n, ray in enumerate(self.range_data):

                # ned position in body frame
                if not (ray > self.max_range or ray < 0.5):
                    
                    points[0,n] = ray*np.cos(n*self.del_theta)            
                    points[1,n] = -ray*np.sin(n*self.del_theta)            
                    points[2,n] = 0            
                    points[3,n] = 1

                    position[0] += ray * np.cos(n*self.del_theta)            
                    position[1] += -ray * np.sin(n*self.del_theta)           
                    position[2] += 0            

                    # compensated = self.transform_mtrx(np.zeros(3),self.datahub.attitude_eular) @ points[:,n]
                    
                    # points[:,n] = compensated

                    vox_n = points[0,n] // self.grid_size
                    vox_e = points[1,n] // self.grid_size
                    vox_d = points[2,n] // self.grid_size
                    
                    # print('this is the position of walls')
                    # self.datahub.vox_n = vox_n
                    # self.datahub.vox_e = vox_e
                    # self.datahub.vox_d = vox_d
                    # print(vox_n*self.grid_size, vox_e*self.grid_size, vox_d*self.grid_size)

                    cnt += 1

                    try:
                        # counting the points in single voxel 
                        downsampled[ (int(vox_n),int(vox_e),int(vox_d)) ] += 1

                    except:    
                        # initialize new point
                        downsampled[ (int(vox_n),int(vox_e),int(vox_d)) ] = 1


            if cnt == 0:
                position = np.array([])
            else:
                position /= cnt


            for coord,num in downsampled.items():

                dist = (coord[0]**2+coord[1]**2+coord[2]**2)**(0.5)

                if dist != 0:

                    if num > self.threshold * round(1/dist):
                        

                        if coord[2] > -2:

                            voxmap_row = -coord[0] + int(len(self.map)/2)
                            voxmap_col =  coord[1] + int(len(self.map)/2)

                            row_ub = voxmap_row + veranda_expension_size 
                            row_lb = voxmap_row - veranda_expension_size 
                            col_ub = voxmap_col + veranda_expension_size 
                            col_lb = voxmap_col - veranda_expension_size 

                            if row_ub > len(self.map):
                                row_ub = len(self.map)-1
                            
                            if col_ub > len(self.map):
                                col_ub = len(self.map)-1

                            if row_lb < 0:
                                row_lb = 0

                            if col_lb < 0:
                                col_lb = 0


                            self.map[row_lb:row_ub,col_lb:col_ub] =\
                            np.ones((row_ub-row_lb,col_ub-col_lb))

            position[3] = 1            

            return self.map,position




    def safety_check(self):

        keepout_vel = np.zeros(3) # ned

        cnt = 0

        for n, ray in enumerate(self.range_data):

            # ned position in body frame
            if 0.5< ray < 3.0:


                keepout_vel[0] += - np.cos(n*self.del_theta)            
                keepout_vel[1] += np.sin(n*self.del_theta)           
                keepout_vel[2] += 0            

                keepout_vel = keepout_vel*(3.0-ray)*0.5
                
                cnt += 1

        if cnt == 0:

            return keepout_vel

        else:

            return keepout_vel/cnt
        




    def generate_grid(self):
        
        self.map = np.zeros((np.shape(self.map)))

        self.map[:,:2] = np.ones((len(self.map),2))
        self.map[:,-2:]= np.ones((len(self.map),2))
        self.map[:2,:] = np.ones((2,len(self.map[0])))
        self.map[-2:,:]= np.ones((2,len(self.map[0])))

        '''
        input : range, angle increment
        output : attitude compensated local map        
        '''

        print(self.map)

        if self.datahub.target_points == None:
            print("not yet")

            return self.map

        else:
            points = np.zeros((4, len(self.datahub.target_points)))
            downsampled = {}

            # for n, ray in enumerate(self.range_data):
            for i,point in enumerate(self.datahub.target_points):

                # ned position in body frame
                # if not (ray > self.max_range or ray < 0.5):
                    
                    # points[0,n] = ray*np.cos(n*self.del_theta)            
                    # points[1,n] = -ray*np.sin(n*self.del_theta)            
                    # points[2,n] = 0            
                    # points[3,n] = 1
                    
                    # compensated = self.transform_mtrx(np.zeros(3),self.datahub.attitude_eular) @ points[:,n]
                    
                    # points[:,n] = compensated

                points[0,i] = point.x             
                points[1,i] = point.y            
                points[2,i] = 0            
                points[3,i] = 1

                compensated = self.transform_mtrx(np.zeros(3),self.datahub.attitude_eular) @ points[:,i]
                
                vox_n = compensated[0] // self.grid_size
                vox_e = compensated[1] // self.grid_size
                vox_d = compensated[2] // self.grid_size
                
                # print('this is the position of walls')
                                
                vox_mean_n.append(vox_n*self.voxel_size)
                vox_mean_e.append(vox_e*self.voxel_size)
                vox_mean_d.append(vox_d*self.voxel_size)

                self.datahub.vox_mean_n = np.sum(vox_mean_n)/len(vox_mean_n)
                self.datahub.vox_mean_e = np.sum(vox_mean_e)/len(vox_mean_e)
                self.datahub.vox_mean_d = np.sum(vox_mean_d)/len(vox_mean_d)

                #print(self.datahub.vox_mean_n,self.datahub.vox_mean_e,self.datahub.vox_mean_d)


                try:
                    # counting the points in single voxel 
                    downsampled[ (int(vox_n),int(vox_e),int(vox_d)) ] += 1

                except:    
                    # initialize new point
                    downsampled[ (int(vox_n),int(vox_e),int(vox_d)) ] = 1


            for coord,num in downsampled.items():

                dist = (coord[0]**2+coord[1]**2+coord[2]**2)**(0.5)

                if dist != 0:

                    if num > self.threshold * round(1/dist):
                        

                        if coord[2] > -2:

                            voxmap_row = -coord[0] + int(len(self.map)/2)
                            voxmap_col =  coord[1] + int(len(self.map)/2)

                            row_ub = voxmap_row + self.expension_size 
                            row_lb = voxmap_row - self.expension_size 
                            col_ub = voxmap_col + self.expension_size 
                            col_lb = voxmap_col - self.expension_size 

                            if row_ub > len(self.map):
                                row_ub = len(self.map)-1
                            
                            if col_ub > len(self.map):
                                col_ub = len(self.map)-1

                            if row_lb < 0:
                                row_lb = 0

                            if col_lb < 0:
                                col_lb = 0


                            self.map[row_lb:row_ub,col_lb:col_ub] =\
                            np.ones((row_ub-row_lb,col_ub-col_lb))

            return self.map


            
