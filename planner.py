import threading
import time
from lib.fsm import FSM
from action_planner import ActionPlanner
import threading
import rospy

class Planner(threading.Thread):

    def __init__(self, drone, datahub):
        super().__init__()

        self.drone = drone
        self.datahub = datahub
        self.state_machine = FSM(self.datahub) # start running sub

    def run(self):

    
        while not self.datahub.is_connected:

            print("Planner : waiting for connection...",end="\r")
            
        print("Planner : connected                 ")
        
        while not rospy.is_shutdown():
            
            # print(f" position ned  : {self.datahub.posvel_ned[:3]}")    
            # print(f" velocity ned  : {self.datahub.posvel_ned[3:]}")    
            # print(f"attitude eular : {self.datahub.attitude_eular}")
            # print(f"  flight mode  : {self.datahub.flight_mode}")
            # print(f"     state     : {self.datahub.state}")
            # print(f"     action    : {self.datahub.action}")
            # print(f"    is armed   : {self.datahub.armed}")
            # print(f"   is in air   : {self.datahub.is_in_air}")
            # print(f"   waypoints   : {self.datahub.waypoints}")
            # print(f" position gps  : {self.datahub.pos_global}")
            self.state_machine.transition()

            time.sleep(0.1)
            


