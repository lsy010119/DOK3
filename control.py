
import numpy as np
import asyncio
import mavsdk
import time
import copy
from lib.trajectory_tracking import TrajectoryTracker
from lib.postion_estmation   import ArUcoPosEstimator
from lib.lidar_processor     import LiDARProcessor 
from lib.search              import Search
from mavsdk.offboard         import OffboardError,\
                                    VelocityNedYaw,\
                                    PositionNedYaw,\
                                    VelocityBodyYawspeed


class Controller:

    def __init__(self, drone, datahub):

        self.drone = drone
        self.datahub = datahub
        self.delt = datahub.delt

        self.lidar_processor = LiDARProcessor(self.datahub)
        self.traj = TrajectoryTracker(self.drone,self.datahub, self.lidar_processor)
        # self.searcher = Search(self.drone,self.datahub)
        self.marker = ArUcoPosEstimator()





    async def arm(self):

        print("Action : armed")
        print("===== Offboard Home Position Set =====")
        
        self.datahub.offboard_home_ned = copy.deepcopy(self.datahub.posvel_ned[:3])
        self.datahub.offboard_home_global = copy.deepcopy(self.datahub.pos_global)
        
        print(f"\nOffboard Home NED    : {self.datahub.offboard_home_ned}")
        print(f"\nOffboard Home Global : {self.datahub.offboard_home_global}")

        await self.drone.action.arm()

        await asyncio.sleep(2)

        self.datahub.state = "Takeoff"
        self.datahub.action = "takeoff"



    async def disarm(self):

        print("Action : disarmed")
        await self.drone.action.disarm()




    async def takeoff(self):

        print("Action : takeoff ...")

        target_alt = - self.datahub.inputs["TARGET_ALTITUDE"] + self.datahub.offboard_home_ned[2]

        print("target alt : ",target_alt)
        destination = np.hstack((self.datahub.posvel_ned[:2],np.array([ target_alt,0,0,0])))
        destination = np.reshape(destination, (6,))
        wp = np.array([])

        await self.traj.trajectory_tracking(destination,wp,1.5,3)

        await asyncio.sleep(3)

        self.datahub.state = "WP"
        self.datahub.action = "tracking"

        # self.datahub.state = "Hold"
        # self.datahub.action = "hold"




    async def hold(self):

        print("Action : hold ...")
        
        hold_position = copy.deepcopy(self.datahub.posvel_ned[:3])
        
        while self.datahub.state == "Hold":


            # keepout_vel = self.lidar_processor.safety_check()

            # await self.drone.offboard.set_velocity_body(
            #     VelocityBodyYawspeed(keepout_vel[0], keepout_vel[1], keepout_vel[2], 0.0))

            yaw = np.rad2deg(self.datahub.attitude_eular[2])

            # await self.drone.offboard.set_velocity_body(
            #     VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw))

            await self.drone.offboard.set_position_ned(
                PositionNedYaw(hold_position[0], hold_position[1], hold_position[2], yaw))

            await asyncio.sleep(0.1)




    async def wp_guidance(self):

        serialized = np.array(self.datahub.inputs["WP"])
    
        self.datahub.v_mean = self.datahub.inputs["MEAN_VELOCITY"] #  pop v_mean

        n = int(len(serialized)/3) # number of the waypoints

        wp = np.zeros((3,n)) # matrix whose columns are the waypoint vectors


        for i in range(n):  

            wp[:,i] = serialized[3*i:3*(i+1)]


        for i in range(3):

            wp[i] += self.datahub.offboard_home_ned[i]

        print("before : ",wp)
        self.datahub.waypoints = wp # update the waypoint data in the datahub





        print("Action : waypoint guidance ...")
        dest_position = np.reshape(self.datahub.waypoints[:,-1],(3,1))
        destination = np.vstack((dest_position,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))

        if len(self.datahub.waypoints[0]) == 1:
            wp = np.array([])

        else:

            wp = self.datahub.waypoints[:,:-1]

        await self.traj.trajectory_tracking(destination,wp,self.datahub.v_mean,self.datahub.traj_update_period)

        self.datahub.waypoints = None
        self.datahub.mission_input = None

        # self.datahub.state = "Hold"
        # self.datahub.action = "hold"
        # self.datahub.state = "Search"
        # self.datahub.action = "search"
        await asyncio.sleep(1)
        self.datahub.state = "Park"
        self.datahub.action = "park"




    async def park(self):
        print("Action : land finding marker 90")
        while True:
            resize_width = np.shape(self.datahub.img_bottom)[1]
            ids,x,y,z = self.marker.run(90,self.datahub.img_bottom,self.datahub.bottom_cam_mtx,self.datahub.bottom_dist_coeff,"DICT_5X5_1000",resize_width)
            if x == {}:
                print("no marker detected")
            else:
                #unit vectorization
                x_distance = x[90]/10
                y_distance = y[90]/10
                z_distance = z[90]/10

                if x_distance < 0.5 and y_distance< 0.5:
                    if z_distance > 7:
                        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.8, 0.0))
                        await asyncio.sleep(2)

                    elif z_distance < 2.0:
                        if x_distance < 0.3 and y_distance< 0.3:
                            await self.drone.action.land()
                        else: pass

                    else:
                        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.8, 0.0))
                        await asyncio.sleep(1)

                print(f'marker detected: {x_distance},{y_distance},{z_distance}')
                unit_x = x_distance / np.linalg.norm([x_distance,y_distance])
                unit_y = y_distance / np.linalg.norm([x_distance,y_distance])

                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(unit_x, unit_y, 0.0, 0.0))
                await asyncio.sleep(0.2)
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            
            


    async def land(self):

        print("Action : land")
        self.datahub.heading_wp += 1
        await self.drone.action.land()




    async def offboard_start(self):

        print("Offboard Starting : Setting initial setpoint ...")

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        print("Offboard Start")

        try:
            await self.drone.offboard.start()

            return True

        except OffboardError as error:

            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")

            return False




    async def search_veranda(self):
        print('search veranda')
        #initialize(delete formal data)
        self.datahub.vox_e = None
        self.datahub.vox_n = None

        #descent 6m
        if self.datahub.SITL == True:
            await self.drone.offboard.set_position_ned(PositionNedYaw(self.datahub.posvel_ned[0], self.datahub.posvel_ned[1], -14, 0.0))
            await asyncio.sleep(3)
            await self.drone.offboard.set_position_ned(PositionNedYaw(self.datahub.posvel_ned[0], self.datahub.posvel_ned[1], -9, 0.0))
            await asyncio.sleep(9)

        else:
            
            while True:
                for point in self.datahub.target_points:
                    z_horrizon = point.z
                if  0.7 < z_horrizon < 1.0:
                    break
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.8, 0.0))
                await asyncio.sleep(0.1)
            self.lidar_processor.generate_grid_real()


        if self.datahub.SITL == True:
            self.lidar_processor.generate_grid_sim()
            print('printed',self.datahub.vox_n)

        else:
            self.lidar_processor.generate_grid_real()
        

        while True:
            #refresh marker detected angle
            
            #marker o
            if self.datahub.cross_marker_detected == True:
                self.datahub.state  = "Crossmarker"
                self.datahub.action = "move_toward_marker"
                break

            # #building x
            # if self.datahub.vox_e == None:
            #     while True:
            #         if self.datahub.vox_e != None:
            #             break
            #         await self.search_descent()
            #     await self.search_align_building()

            #building o, marker x                                    무조건 건물보다 위에 있다고 가정했을때 첫 align은 안해도 됨 
            elif self.datahub.cross_marker_detected == False:
                await self.search_align_building()
                await self.search_circle_move(self.datahub.circle_move_degree)
                await self.search_descent()
                


    #마커를 찾을 때까지 돈다(못찾으면 한 바퀴까지만 돈다.)
    async def search_circle_move(self,circle_move_degree):
        #yaw_move
        circle_start_degree = self.datahub.circle_yaw_angle
        while True:
            if self.datahub.cross_marker_detected == True:
                self.datahub.first_marker_detected_angle = copy.deepcopy(self.datahub.circle_yaw_angle)
                break

            #counter clock wise circle move
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, -circle_move_degree))
            await asyncio.sleep(0.2)
            print(f"circle_start_degree:{circle_start_degree}, self.datahub.circle_yaw_angle:{self.datahub.circle_yaw_angle} ")
            if -0.5 < (circle_start_degree - self.datahub.circle_yaw_angle) < 0:  #+1은 코드 돌아가는 시간 추가
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(1)
                break
        

    #0.8m/s로 1초 하강 후 라이다 거리정보 평균화
    async def search_descent(self):
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 1.0, 0.0))
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)
        if self.datahub.SITL == True:
            self.lidar_processor.generate_grid_sim()
            print(self.datahub.vox_n,"qwerqwerqwerqwerqwerqwer")

        else:
            self.lidar_processor.generate_grid_real()
        

    #align to building
    async def search_align_building(self):
        print('align to building')
        if self.datahub.SITL == True:
            self.lidar_processor.generate_grid_sim()

        else:
            self.lidar_processor.generate_grid_real()
        circle_center_X = self.datahub.vox_e
        circle_center_Y = self.datahub.vox_n      
        print(f'circle_center_Y, circle_center_X: {circle_center_Y, circle_center_X}')
        circle_center_degree_ned = ((np.arctan2(circle_center_Y,circle_center_X))* 180/np.pi)-90
        print(f'circle_center_degree_ned: {circle_center_degree_ned}')
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(3)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, -circle_center_degree_ned))
        await asyncio.sleep(3)
        radius = np.linalg.norm([self.datahub.vox_n, self.datahub.vox_e])
        circle_move_degree = (1/radius)*180/np.pi

        self.datahub.circle_radius      = circle_move_degree
        self.datahub.circle_move_degree = radius
        print(f'degree:{circle_move_degree}')
        print(f"circle mode {radius}m radius")



    # #align to marker -> only yaw rotation, circle search
    # async def search_align_marker(self):
    #     pass




    async def move_toward_marker(self):
        print('marker detected')
        print(self.datahub.cross_marker)
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)


        if self.datahub.SITL == True:
            self.lidar_processor.generate_grid_sim()

        if self.datahub.SITL == False:
            self.lidar_processor.generate_grid_real()
             
        while True:
            
            print(np.linalg.norm([self.datahub.vox_n, self.datahub.vox_e]))
            if np.linalg.norm([self.datahub.vox_n, self.datahub.vox_e]) < 2.2:
                ##################
                #####여기서 스테이트 바꿈 투하로
                ################## 여기서는 잠깐 홀드로 함 원래 ejection으로 바꿔야함
                self.datahub.state  = "Hold"
                self.datahub.action = "hold"
                self.datahub.mission_input = None
                break
            #
            # if (self.datahub.cross_marker[0] - self.datahub.img_center_sim[0] > 20) and (self.datahub.cross_marker[1] - self.datahub.img_center_sim[1] > 20):
            #     await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -0.3, -0.3, 0.0))
            #     asyncio.sleep(0.2)
            #     await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            #     asyncio.sleep(0.2)
            #     if self.
            #         await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))
            #         await asyncio.sleep(0.2)

            if self.datahub.SITL == True:
                self.lidar_processor.generate_grid_sim()

            else:
                self.lidar_processor.generate_grid_real()

            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)
