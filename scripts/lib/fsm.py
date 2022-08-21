import rospy
import numpy as np
from std_msgs.msg import String
import time

class FSM:
    def __init__(self, datahub):

        self.datahub = datahub

        # 1. Disarm
        # 2. Arm 
        # 3. Takeoff
        # 4. Hold
        # 5. Trajectory
        # 6. PrepareEject
        # 7. Eject
        # 8. PrepareLand
        # 9. Land
        # 10. EmergencyStop

        # State
        self.state_list = {}
        self.state_list["Disarm"]        = Disarm(self.datahub)
        self.state_list["Arm"]           = Arm(self.datahub)
        self.state_list["Takeoff"]       = Takeoff(self.datahub)
        self.state_list["Hold"]          = Hold(self.datahub)
        self.state_list["WP"]            = Trajectory(self.datahub)
        self.state_list["PrepareLand"]   = PrepareLand(self.datahub)
        self.state_list["Land"]          = Land(self.datahub)
        self.state_list["Park"]          = Park(self.datahub)
        self.state_list["EmergencyStop"] = EmergencyStop(self.datahub)

        # Current state
        self.on_going_state = None

        # Mission input
        rospy.Subscriber("/mission_input", String, self.mission_callback)

    def mission_callback(self, msg):
        self.datahub.mission_input = msg.data

    def transition(self):
        self.on_going_state = self.state_list[self.datahub.state]
        self.on_going_state.transition()



# in datahub(shared), there is all sensor data.
class State:
    def __init__(self, datahub):
        self.datahub = datahub

    # Change next state here
    def transition(self):
        return NotImplementedError()



class Disarm(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if self.datahub.mission_input == "Arm":
            self.datahub.state = "Arm"

            self.datahub.action = "arm"

        else:
            pass



class Arm(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if not self.datahub.armed:
            self.datahub.state = "Disarm"

        elif self.datahub.mission_input == "Disarm":
            self.datahub.state = "Disarm"

            self.datahub.action = "disarm"

        elif self.datahub.mission_input == "Takeoff":
            self.datahub.state = "Takeoff"

            self.datahub.action = "takeoff"

        else:
            pass
    


class Takeoff(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if -self.datahub.posvel_ned[2] >= -self.datahub.waypoints[2]:

            self.datahub.state = "Hold"

            self.datahub.action = "hold"


        else:   
            pass



class Hold(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        
        if self.datahub.mission_input == "Land":
            self.datahub.state = "Land"

            self.datahub.action = "land"

        elif self.datahub.mission_input == "Park":
            self.datahub.state = "Park"

            self.datahub.action = "park"

        elif self.datahub.mission_input == "WP":
            self.datahub.state = "WP"

            self.datahub.action = "tracking"


        else:
            pass
        


class Trajectory(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if np.linalg.norm(self.datahub.posvel_ned[:3] - self.datahub.waypoints[:,-1]) <= 1:

            self.datahub.state = "Hold"

            self.datahub.action = "hold"


        else:   
            pass   
    


class PrepareLand(State):
    def __init__(self, datahub):
        super().__init__(datahub)

        self.token = 0

    def transition(self):
        
        ## search marker
        if self.datahub.marker_detected:
            ## track to marker's position
            if np.linalg.norm(self.datahub.position - self.datahub.marker_position):
                self.datahub.marker_detected = False
                self.next_state = "Land"
                self.datahub.state = self.next_state

        else:
            pass
        


class Land(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        
        if -self.datahub.posvel_ned[2] <= 0.0:
            self.datahub.state = "Disarm"

            # self.datahub.action = "disarm"

        else:   
            pass



class Park(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        
        if -self.datahub.posvel_ned[2] <= 0.0:
            self.datahub.state = "Disarm"

            # self.datahub.action = "disarm"

        else:   
            pass



class EmergencyStop(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        pass
