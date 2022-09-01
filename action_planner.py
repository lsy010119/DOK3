import asyncio
import numpy as np
from control        import Controller


class ActionPlanner:

    def __init__(self, drone, datahub):
        
        self.drone = drone
        self.datahub = datahub
        
        self.control = Controller(self.drone, self.datahub)
   

    async def run(self):

        while not self.datahub.is_connected:

            # waiting for connection...

            await asyncio.sleep(0.01)


        await asyncio.sleep(3)


        while self.datahub.is_connected:


            if self.datahub.action == "arm":

                self.datahub.action = None
                await self.control.arm()




            if self.datahub.action == "disarm":
            
                self.datahub.action = None
                await self.control.disarm()
            



            if self.datahub.action == "takeoff":
            
                self.datahub.action = None

                await self.control.offboard_start()
                await self.control.takeoff()
                



            if self.datahub.action == "hold":
                
                self.datahub.action = None
                await self.control.hold()                
            



            if self.datahub.action == "land":
            
                self.datahub.action = None
                await self.control.land()
            



            if self.datahub.action == "park":
            
                self.datahub.action = None
                await self.control.park()




            if self.datahub.action == "tracking":
                
                self.datahub.action = None
                await self.control.wp_guidance()
            
            

            
            if self.datahub.action == "search":
           
                self.datahub.action = None
                await self.control.search_veranda()
            

            
            
            if self.datahub.action == 'move_toward_marker':

                self.datahub.action = None
                await self.control.move_toward_marker()
            

            
            await asyncio.sleep(0.01)
        
