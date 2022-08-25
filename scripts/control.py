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

        # self.lidar_processor = LiDARProcessor(self.datahub)
        self.traj = TrajectoryTracker(self.drone,self.datahub)
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




    async def disarm(self):

        print("Action : disarmed")
        await self.drone.action.disarm()




    async def takeoff(self):

        print("Action : takeoff ...")

        destination = np.vstack((self.datahub.waypoints,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))
        wp = np.array([])

        await self.traj.trajectory_tracking(destination,wp,1)


        self.datahub.state = "Hold"
        self.datahub.action = "hold"




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

        print("Action : waypoint guidance ...")
        dest_position = np.reshape(self.datahub.waypoints[:,-1],(3,1))
        destination = np.vstack((dest_position,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))

        if len(self.datahub.waypoints[0]) == 1:
            wp = np.array([])

        else:

            wp = self.datahub.waypoints[:,:-1]

        await self.traj.trajectory_tracking(destination,wp,self.datahub.v_mean)

        self.datahub.waypoints = None
        self.datahub.mission_input = None

        self.datahub.state = "Hold"
        self.datahub.action = "hold"




    async def park(self):

        print("Action : park ...")
        
        cnt = 0

        while cnt < 30:
            resize_width = np.shape(self.datahub.img_bottom)[1]

            # Save your OpenCV2 image as a jpeg 
            ids,x,y,z = self.marker.run(self.datahub.img_bottom,self.cam_mtx,self.dist_coeff,"DICT_5X5_1000",resize_width)
            try:
                
                body_pos = self.traj.ned2xyz(self.datahub.posvel_ned[:3])                
                
                marker_pos = np.zeros((6,))

                marker_pos[0] = body_pos[0] + x[95]*0.002
                marker_pos[1] = body_pos[1] - y[95]*0.002
                # marker_pos[0] = 0
                # marker_pos[1] = 0
                marker_pos[2] = 0

                marker_pos = self.traj.xyz2ned(marker_pos)

                wp = np.array([])

                print(marker_pos[:3])

                await self.traj.trajectory_tracking(marker_pos,wp,1)
                await self.drone.action.land()
                break

            except:

                await asyncio.sleep(0.1)
                time.sleep(0.1)
                cnt += 1

        self.datahub.state = "Land"
        self.datahub.action = "land"
        self.datahub.mission_input = None




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

        print('Action : search veranda')

        await self.searcher.run()







    async def move_toward_marker(self):
        print('marker detected')
        print(self.datahub.cross_marker)
        
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 2.0, 0.0))
        await asyncio.sleep(5)
