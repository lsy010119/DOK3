#!/usr/bin/python3
import asyncio

from mavsdk             import System
from mavsdk.offboard    import OffboardError,VelocityNedYaw,PositionNedYaw


class Connector:

    def __init__(self, drone, datahub):


        self.drone = drone

        self.datahub = datahub


    async def connect(self):
        '''
        
        system_address="udp://:14540" : Connect with simulator

        system_address="serial://<ttyUSB>:<baudrate>" : Serial Connect  
        
        '''

        await self.drone.connect(system_address="udp://:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("connected                    ",end="\r")
                self.datahub.is_connected = True
                break


    async def offboard_start(self):


        await self.drone.action.arm()
        print("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")

        try:
            await self.drone.offboard.start()
            return True

        except OffboardError as error:

            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")

            print("-- Disarming")
            await self.drone.action.disarm()   
            return False
