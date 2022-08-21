from cvxpy import length
import numpy as np
import matplotlib.pyplot as plt     
import rospy
import time   

from matplotlib import colors 
from std_msgs.msg import Float32MultiArray


class MapListner:

    def __init__(self):

        rospy.init_node("alzartakarsen")

        rospy.Subscriber("/map",Float32MultiArray,self.callback)

        self.map = np.zeros((151,151))

        self.vel = np.ones(2)


    def callback(self,msg):

        map_data = msg.data

        self.vel = map_data[-2:]

        map_data = np.array(map_data[:-2])

        width = int(len(map_data)**(0.5))

        self.map = np.reshape(map_data,(width,width))


    def run(self):

        while not rospy.is_shutdown():

            plt.cla()
            # print("running",time.time())
            cmap= colors.ListedColormap(['white','black','red','green'])
            plt.imshow(self.map,cmap)
            plt.title("JPS Algorithm")
            plt.quiver(len(self.map)/2,len(self.map)/2,self.vel[1],self.vel[0])
            plt.axis('off')
            plt.pause(0.5)

        plt.show()


if __name__ == "__main__":

    ML = MapListner()

    ML.run()