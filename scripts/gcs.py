import rospy
import numpy as np

from std_msgs.msg import String,Float32MultiArray

class GCS:

    def __init__(self):
        
        rospy.init_node("gcs")
        self.mission_pub = rospy.Publisher("/mission_input",String,queue_size=1)
        self.waypoint_pub = rospy.Publisher("/waypoints_input",Float32MultiArray,queue_size=1)
        # self.input_avaliable = True
        self.avaliable_mission = ["Arm","Disarm","Takeoff","Land","Park","WP"]

    def run(self):

        while not rospy.is_shutdown():

            # if self.input_avaliable:

            mission = input("mission : ")

            if mission in self.avaliable_mission:
                
                if mission == "Takeoff":

                    target_alt = float(input("altitude : "))
                    
                    waypoint = Float32MultiArray()

                    waypoint.data = np.array([1,0,0,-target_alt])

                    self.waypoint_pub.publish(waypoint)
                    self.mission_pub.publish(mission)
                    

                elif mission == "WP":

                    n = int(input("number of waypoints : "))
                    
                    waypoints = Float32MultiArray()

                    v_mean = float(input("mean velocity : "))

                    waypoint_mtrx = np.array([v_mean])
                    

                    print("input the coordinates of the waypoints in NED frme")

                    for i in range(n): 

                        wp_input = input("wp #%d : " % (i+1)).split(" ")

                        waypoint = np.array([float(wp_input[0]),float(wp_input[1]),float(wp_input[2])])

                        waypoint_mtrx = np.hstack((waypoint_mtrx,waypoint))

                    waypoints.data = waypoint_mtrx
                    self.waypoint_pub.publish(waypoints)
                    self.mission_pub.publish(mission)


                else:

                    self.mission_pub.publish(mission)
                
            
            else:
                
                print("Not valid mission")
                continue

        

if __name__ == "__main__":

    G = GCS()
    G.run()
