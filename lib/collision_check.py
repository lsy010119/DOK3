import numpy as np
import matplotlib.pyplot as plt     
import time   
from matplotlib import colors 


class CollisionChecker:

    def __init__(self):
        pass

    def f(self,p1,p2,x):

        try:
            return int( ( (p2[1]-p1[1])/(p2[0]-p1[0]) )*( x - p1[0] ) + p1[1] )
        except:
            pass


    def check_collision(self, p1, p2, map):
        

        collision = False

        for i in range(10*abs(p2[0]-p1[0])):

            if p1[0] < p2[0]:
                x = p1[0]+0.1*i+1
            else:
                x = p1[0]-0.1*i+1

            y = self.f(p1,p2,x)

            if map[int(x),int(y)] == 1:
                collision = True

        return collision
