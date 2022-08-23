from tracemalloc import start
import numpy as np
import matplotlib.pyplot as plt     
import time   
from matplotlib import colors 

class Node:

    def __init__(self, row, col, parent_node=None, cornor_dir=None):

        self.row = row
        self.col = col        
        self.Parent = parent_node

        self.dist_cost = 0

        self.dynamical_cost = 0

        self.total_cost = 0

        self.cornor_dir = cornor_dir 
        # for cornor node, to classify the direction of the cornor so that i can decide new search point
        # UR, UL, RU, RD, DL, DL, LU, LD

    def cost(self, start_point, goal_point, weight_dir, velocity):

        # Euclidean distance heuristic
        cost_g = (self.row - start_point[0])**2 + (self.col - start_point[1])**2
        cost_h = (self.row - goal_point[0])**2 + (self.col - goal_point[1])**2
        self.dist_cost = cost_g + cost_h



        # Motion primitive
        dir_vec = np.array([ self.Parent.row - self.row, self.col - self.Parent.col ])

        self.dynamical_cost = (velocity**2) * 1000 * np.arccos((weight_dir @ dir_vec )/(np.linalg.norm(weight_dir)*np.linalg.norm(dir_vec))) / np.pi



        self.total_cost = self.dist_cost + self.dynamical_cost

        return self.total_cost

        


class SearchPlanner:

    def __init__(self,grid_map):

        self.map = grid_map                                 # map
        
        self.visited_area = np.zeros(np.shape(grid_map))    # visited area

        


    '''Check functions'''

    def is_UR_blocked(self, node):
        node_r = node.row
        node_c = node.col
    
        is_UR_inwall = (self.visited_area[node_r-1,node_c+1] == 1) or (self.map[node_r-1,node_c+1] == 1)

        is_thinwall = (self.map[node_r-1,node_c] == 1) or (self.map[node_r,node_c+1] == 1)

        return is_UR_inwall or is_thinwall


    def is_DR_blocked(self, node):
        node_r = node.row
        node_c = node.col

        is_DR_inwall = (self.visited_area[node_r+1,node_c+1] == 1) or (self.map[node_r+1,node_c+1] == 1)

        is_thinwall = (self.map[node_r+1,node_c] == 1) or (self.map[node_r,node_c+1] == 1)
        
        return is_DR_inwall or is_thinwall


    def is_DL_blocked(self, node):
        node_r = node.row
        node_c = node.col

        is_DL_inwall = (self.visited_area[node_r+1,node_c-1] == 1) or (self.map[node_r+1,node_c-1] == 1)

        is_thinwall = (self.map[node_r+1,node_c] == 1) or (self.map[node_r,node_c-1] == 1)
        
        return is_DL_inwall or is_thinwall


    def is_UL_blocked(self, node):
        node_r = node.row
        node_c = node.col

        is_UL_inwall = (self.visited_area[node_r-1,node_c-1] == 1) or (self.map[node_r-1,node_c-1] == 1)

        is_thinwall = (self.map[node_r-1,node_c] == 1) or (self.map[node_r,node_c-1] == 1)
        
        return is_UL_inwall or is_thinwall


    def no_way_to_go(self, node):
        
        node_r = node.row
        node_c = node.col

        # check if it is all blocked
        return  (self.visited_area[node_r-1,node_c+1] == 1 or self.map[node_r-1,node_c+1] == 1) and \
                (self.visited_area[node_r-1,node_c-1] == 1 or self.map[node_r-1,node_c-1] == 1) and \
                (self.visited_area[node_r+1,node_c+1] == 1 or self.map[node_r+1,node_c+1] == 1) and \
                (self.visited_area[node_r+1,node_c-1] == 1 or self.map[node_r+1,node_c-1] == 1) and \
                ((self.visited_area[node_r-1,node_c] == 1 or self.map[node_r-1,node_c] == 1) and \
                 (self.visited_area[node_r,node_c+1] == 1 or self.map[node_r,node_c+1] == 1))   and\
                ((self.visited_area[node_r-1,node_c] == 1 or self.map[node_r-1,node_c] == 1) and \
                 (self.visited_area[node_r,node_c-1] == 1 or self.map[node_r,node_c-1] == 1))   and\
                ((self.visited_area[node_r+1,node_c] == 1 or self.map[node_r+1,node_c] == 1) and \
                 (self.visited_area[node_r,node_c+1] == 1 or self.map[node_r,node_c+1] == 1))   and\
                ((self.visited_area[node_r+1,node_c] == 1 or self.map[node_r+1,node_c] == 1) and \
                 (self.visited_area[node_r,node_c-1] == 1 or self.map[node_r,node_c-1] == 1))



    ''' Basic Single Direction Search Functions '''
    
    def upward_search(self,node, cornor_list):
    
        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        '''upper search'''
        i = 0
        self.visited_area[node_r,node_c] = 0

        while (self.visited_area[node_r-i,node_c] != 1) and (self.map[node_r-i,node_c] != 1):


            # check left forced neighbor
            if ( self.map[node_r-i,node_c-1] == 1 ) and ( self.map[node_r-i-1,node_c-1] != 1 ) and \
                (( self.map[node_r-i-1,node_c] != 1 ) and ( self.visited_area[node_r-i-1,node_c] != 1 )):

                # print("corner detected: u_L")
                self.visited_area[node_r-i,node_c] = 1

                cornor_node = Node(node_r-i-1,node_c,node,"UL") # cornor node whose parents is intput node

                cornor_list.append(cornor_node) # append the cornor node
                
                self.visited_area[node_r-i-1,node_c] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node


            # check right forced neighbor
            if ( self.map[node_r-i,node_c+1] == 1 ) and ( self.map[node_r-i-1,node_c+1] != 1 ) and \
                (( self.map[node_r-i-1,node_c] != 1 ) and ( self.visited_area[node_r-i-1,node_c] != 1 )):

                # print("corner detected: u_R")
                self.visited_area[node_r-i,node_c] = 1

                cornor_node = Node(node_r-i-1,node_c,node,"UR") 
                
                cornor_list.append(cornor_node) # append the cornor node
                
                self.visited_area[node_r-i-1,node_c] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node
                
            self.visited_area[node_r-i,node_c] = 1
            
            i += 1


    def right_search(self,node, cornor_list):

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        '''right search'''
        i = 0
        self.visited_area[node_r,node_c] = 0

        while (self.visited_area[node_r,node_c+i] != 1) and (self.map[node_r,node_c+i] != 1):
                

            # check upper forced neighbor
            if ( self.map[node_r-1,node_c+i] == 1 ) and ( self.map[node_r-1,node_c+i+1] != 1 ) and \
                (( self.map[node_r,node_c+i+1] != 1 ) and ( self.visited_area[node_r,node_c+i+1] != 1 )):

                # print("corner detected: r_U")
                self.visited_area[node_r,node_c+i] = 1

                cornor_node = Node(node_r,node_c+i+1,node,"RU") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r,node_c+i+1] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node


            # check downward forced neighbor
            if ( self.map[node_r+1,node_c+i] == 1 ) and ( self.map[node_r+1,node_c+i+1] != 1 ) and \
                (( self.map[node_r,node_c+i+1] != 1 ) and ( self.visited_area[node_r,node_c+i+1] != 1 )):

                # print("corner detected: r_D")
                self.visited_area[node_r,node_c+i] = 1

                cornor_node = Node(node_r,node_c+i+1,node,"RD") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r,node_c+i+1] = 1
                
                break
                ### new search starts at cornor, set the parent of the corner node as search node

            
            # set current position as visited
            self.visited_area[node_r,node_c+i] = 1

            i += 1


    def downward_search(self,node, cornor_list):
    
        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        '''downward search'''
        i = 0
        self.visited_area[node_r,node_c] = 0

        while (self.visited_area[node_r+i,node_c] != 1) and (self.map[node_r+i,node_c] != 1):

            # check left forced neighbor
            if ( self.map[node_r+i,node_c-1] == 1 ) and ( self.map[node_r+i+1,node_c-1] != 1 ) and \
                (( self.map[node_r+i+1,node_c] != 1 ) and ( self.visited_area[node_r+i+1,node_c] != 1 )):

                # print("corner detected: d_L")
                self.visited_area[node_r+i,node_c] = 1

                cornor_node = Node(node_r+i+1,node_c,node,"DL") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r+i+1,node_c] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node
                

                

            # check right forced neighbor
            if ( self.map[node_r+i,node_c+1] == 1 ) and ( self.map[node_r+i+1,node_c+1] != 1 ) and \
                (( self.map[node_r+i+1,node_c] != 1 ) and ( self.visited_area[node_r+i+1,node_c] != 1 )):

                # print("corner detected: d_R")
                self.visited_area[node_r+i,node_c] = 1

                cornor_node = Node(node_r+i+1,node_c,node,"DR") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r+i+1,node_c] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node
                



            # set current position as visited
            self.visited_area[node_r+i,node_c] = 1
            
            i += 1


    def left_search(self,node, cornor_list):
    
        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        '''left search'''
        i = 0
        self.visited_area[node_r,node_c] = 0  

        while (self.visited_area[node_r,node_c-i] != 1) and (self.map[node_r,node_c-i] != 1):


            # check upper forced neighbor
            if ( self.map[node_r-1,node_c-i] == 1 ) and ( self.map[node_r-1,node_c-i-1] != 1 ) and \
                (( self.map[node_r,node_c-i-1] != 1 ) and ( self.visited_area[node_r,node_c-i-1] != 1 )):

                # print("corner detected: l_U")
                self.visited_area[node_r,node_c-i] = 1

                cornor_node = Node(node_r,node_c-i-1,node,"LU") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r,node_c-i-1] = 1

                break
                ### new search starts at cornor, set the parent of the corner node as search node
                

                
    
            # check downward forced neighbor
            if ( self.map[node_r+1,node_c-i] == 1 ) and ( self.map[node_r+1,node_c-i-1] != 1 ) and \
                (( self.map[node_r,node_c-i-1] != 1 ) and ( self.visited_area[node_r,node_c-i-1] != 1 )):

                # print("corner detected: l_D")
                self.visited_area[node_r,node_c-i] = 1

                cornor_node = Node(node_r,node_c-i-1,node,"LD") 
                
                cornor_list.append(cornor_node)
                
                self.visited_area[node_r,node_c-i-1] = 1

                break                
                ### new search starts at cornor, set the parent of the corner node as search node
                

            # set current position as visited
            self.visited_area[node_r,node_c-i] = 1

            i += 1

    

    ''' Single Quadrant Search Functions '''

    def UR_search(self,node, cornor_list):

        ''' Up-Right Quadrant Search '''

        # print("UR Quadrant Searching")

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        i = 0

        while True: # until no more way to move left

            UR_node = Node(node_r-i,node_c+i,node)

            self.upward_search(UR_node, cornor_list)

            self.right_search(UR_node, cornor_list)


            if self.is_UR_blocked(UR_node):
                
                break 

            i += 1


    def DR_search(self,node, cornor_list):

        ''' Down-Right Quadrant Search '''

        # print("DR Quadrant Searching")

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        i = 0

        while True: # until no more way to move left

            DR_node = Node(node_r+i,node_c+i,node)

            self.right_search(DR_node, cornor_list)

            self.downward_search(DR_node, cornor_list)
            

            if self.is_DR_blocked(DR_node):
                
                break

            i += 1


    def DL_search(self,node, cornor_list):

        ''' Down-Left Quadrant Search '''

        # print("DL Quadrant Searching")

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        i = 0

        while True: # until no more way to move left

            DL_node = Node(node_r+i,node_c-i,node)

            self.downward_search(DL_node, cornor_list)
            
            self.left_search(DL_node, cornor_list)


            if self.is_DL_blocked(DL_node):

                break

            i += 1


    def UL_search(self,node, cornor_list):

        ''' Up-Left Quadrant Search '''

        # print("UL Quadrant Searching")

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        i = 0

        while True: # until no more way to move left

            UL_node = Node(node_r-i,node_c-i,node)

            self.upward_search(UL_node, cornor_list)
            
            self.left_search(UL_node, cornor_list)


            if  self.is_UL_blocked(UL_node):

                break

            i += 1





    def search_cornor(self, node, CW):

        cornor_list = [] # initial cornor list which contains the cornors

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node


        ''' search for cornors '''
        # print("UR quad")
        self.UR_search(node, cornor_list)

        # print("DR quad")
        self.DR_search(node, cornor_list)

        # print("DL quad")
        self.DL_search(node, cornor_list)

        # print("UL quad")
        self.UL_search(node, cornor_list)

        ''' repeat for cornor with minimal cost if no goal detected '''

        # side_cornor = 
        # side_cornor_idx = 0

        if len(cornor_list) == 0:

            return np.array([])
        
        else:

            cornor_row = []
            cornor_col = []

            # find cornor 

            for cornor in cornor_list:

                cornor_row.append(cornor.row)
                cornor_col.append(cornor.col)

            if CW:
                row = max(cornor_row)
                col = min(cornor_col)

            else:
                row = max(cornor_row)
                col = max(cornor_col)

            side_cornor = np.array([int(len(self.map)//2)-row, col-int(len(self.map)//2), 0, 1])

            return side_cornor



    def run(self,CW):

        search_node = Node(int(len(self.map)//2),int(len(self.map)//2),None) 
        return self.search_cornor(search_node,CW)
