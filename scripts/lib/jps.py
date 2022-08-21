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

class JPS:

    def __init__(self,grid_map,start,goal,vel_vec):

        self.map = grid_map                                 # map
        
        self.visited_area = np.zeros(np.shape(grid_map))    # visited area
        
        self.start = start                                  # start point

        self.vel_vec = vel_vec                              # current velocity vectory for calculating dynamical cost
        
        self.vel_size = np.linalg.norm(vel_vec)

        c = int( len(self.map)//2 ) # half of the length of the side

        # check if the goal point locates out of the map
        if not ((1 < goal[0] < len(self.map)-2) and (1 < goal[1] < len(self.map)-2)):

            # row for col = 2   
            try:
                row_for_col_2 = int( (((c-goal[0])/(c-goal[1]))*(2-c)+c) )

                row_for_col_e = int( (((c-goal[0])/(c-goal[1]))*(len(self.map)-3-c)+c) )
            
            except:
            
                row_for_col_2 = 15

                row_for_col_e = 15
                            
            try:

                col_for_row_2 = int( (((c-goal[1])/(c-goal[0]))*(2-c)+c) )

                col_for_row_e = int( (((c-goal[1])/(c-goal[0]))*(len(self.map)-3-c)+c) )

            except:

                col_for_row_2 = 15

                col_for_row_e = 15

            if goal[1] < 2:

                if 1 < row_for_col_2 < len(self.map)-2:

                    self.goal = np.array([row_for_col_2, 2])


            if goal[1] > len(self.map) - 3:

                if 1 < row_for_col_e < len(self.map)-2:

                    self.goal = np.array([row_for_col_e, len(self.map)-3])


            if goal[0] < 2:

                if 1 < col_for_row_2 < len(self.map)-2:

                    self.goal = np.array([2, col_for_row_2])


            if goal[0] > len(self.map) - 3:

                if 1 < col_for_row_e < len(self.map)-2:

                    self.goal = np.array([len(self.map)-3, col_for_row_e])


        else:          
            
            self.goal = goal                                    # end point
        
        # print(f"in JPS {self.goal}")

        self.closedlist = []


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
            # print(self.visited_area)
            # time.sleep(0.1)
            if self.goal[0] == node_r-i and self.goal[1] == node_c:
                
                # print(f"goal founded!!!!!!!!!!!!!!!!!!!")
                self.closedlist.append(node)
                return True
                

            # check left forced neighbor
            if ( self.map[node_r-i,node_c-1] == 1 ) and ( self.map[node_r-i-1,node_c-1] != 1 ) and \
                (( self.map[node_r-i-1,node_c] != 1 ) and ( self.visited_area[node_r-i-1,node_c] != 1 )):

                # print("corner detected: u_L")
                self.visited_area[node_r-i,node_c] = 1

                cornor_node = Node(node_r-i-1,node_c,node,"UL") # cornor node whose parents is intput node

                cornor_list.append(cornor_node) # append the cornor node

                if self.goal[0] == node_r-i-1 and self.goal[1] == node_c:
                    return True
                
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

                if self.goal[0] == node_r-i-1 and self.goal[1] == node_c:
                    return True
                
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
            # print(self.visited_area)
            # time.sleep(0.1)
            if self.goal[0] == node_r and self.goal[1] == node_c+i:
                
                # print(f"goal founded!!!!!!!!!!!!!!!!!!!")
                # self.closedlist.append(node)
                self.closedlist.append(node)

                return True
                

            # check upper forced neighbor
            if ( self.map[node_r-1,node_c+i] == 1 ) and ( self.map[node_r-1,node_c+i+1] != 1 ) and \
                (( self.map[node_r,node_c+i+1] != 1 ) and ( self.visited_area[node_r,node_c+i+1] != 1 )):

                # print("corner detected: r_U")
                self.visited_area[node_r,node_c+i] = 1

                cornor_node = Node(node_r,node_c+i+1,node,"RU") 
                
                cornor_list.append(cornor_node)

                if self.goal[0] == node_r and self.goal[1] == node_c+i+1:
                    return True
                
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

                if self.goal[0] == node_r and self.goal[1] == node_c+i+1:
                    return True
                
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
            # print(self.visited_area)
            # time.sleep(0.1)
            if self.goal[0] == node_r+i and self.goal[1] == node_c:

                # print(f"goal founded!!!!!!!!!!!!!!!!!!!")
                self.closedlist.append(node)
                return True
                

            # check left forced neighbor
            if ( self.map[node_r+i,node_c-1] == 1 ) and ( self.map[node_r+i+1,node_c-1] != 1 ) and \
                (( self.map[node_r+i+1,node_c] != 1 ) and ( self.visited_area[node_r+i+1,node_c] != 1 )):

                # print("corner detected: d_L")
                self.visited_area[node_r+i,node_c] = 1

                cornor_node = Node(node_r+i+1,node_c,node,"DL") 
                
                cornor_list.append(cornor_node)

                if self.goal[0] == node_r+i+1 and self.goal[1] == node_c:
                    return True
                
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

                if self.goal[0] == node_r+i+1 and self.goal[1] == node_c:
                    return True
                
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
            # print(self.visited_area)
            # time.sleep(0.1)
            if self.goal[0] == node_r and self.goal[1] == node_c-i:
                
                # print(f"goal founded!!!!!!!!!!!!!!!!!!!")
                self.closedlist.append(node)
                return True
                

            # check upper forced neighbor
            if ( self.map[node_r-1,node_c-i] == 1 ) and ( self.map[node_r-1,node_c-i-1] != 1 ) and \
                (( self.map[node_r,node_c-i-1] != 1 ) and ( self.visited_area[node_r,node_c-i-1] != 1 )):

                # print("corner detected: l_U")
                self.visited_area[node_r,node_c-i] = 1

                cornor_node = Node(node_r,node_c-i-1,node,"LU") 
                
                cornor_list.append(cornor_node)

                if self.goal[0] == node_r and self.goal[1] == node_c-i-1:
                    return True
                
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

                if self.goal[0] == node_r and self.goal[1] == node_c-i-1:
                    return True
                
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

            # if goal is detected, than append the UR_node whose parent is node 

            if self.upward_search(UR_node, cornor_list):

                # self.closedlist.append(UR_node)

                return True

            if self.right_search(UR_node, cornor_list):

                # self.closedlist.append(UR_node)

                return True

            # if self.downward_search(UR_node, cornor_list):

            #     # self.closedlist.append(UR_node)

            #     return True
            
            # if self.left_search(UR_node, cornor_list):

            #     # self.closedlist.append(UR_node)

            #     return True

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

            # if goal is detected, than append the DR_node whose parent is node 

            # if self.upward_search(DR_node, cornor_list):

            #     # self.closedlist.append(DR_node)

            #     return True

            if self.right_search(DR_node, cornor_list):

                # self.closedlist.append(DR_node)

                return True

            if self.downward_search(DR_node, cornor_list):

                # self.closedlist.append(DR_node)

                return True
            
            # if self.left_search(DR_node, cornor_list):

            #     # self.closedlist.append(DR_node)

            #     return True

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

            # if goal is detected, than append the DL_node whose parent is node 

            # if self.upward_search(DL_node, cornor_list):

            #     # self.closedlist.append(DL_node)

            #     return True

            # if self.right_search(DL_node, cornor_list):

            #     # self.closedlist.append(DL_node)

            #     return True

            if self.downward_search(DL_node, cornor_list):

                # self.closedlist.append(DL_node)

                return True
            
            if self.left_search(DL_node, cornor_list):

                # self.closedlist.append(DL_node)

                return True

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

            # if goal is detected, than append the UL_node whose parent is node 

            if self.upward_search(UL_node, cornor_list):

                # self.closedlist.append(UL_node)

                return True

            # if self.right_search(UL_node, cornor_list):

            #     # self.closedlist.append(UL_node)

            #     return True

            # if self.downward_search(UL_node, cornor_list):

            #     # self.closedlist.append(UL_node)

            #     return True
            
            if self.left_search(UL_node, cornor_list):

                # self.closedlist.append(UL_node)

                return True

            if  self.is_UL_blocked(UL_node):

                break

            i += 1



    def search(self, node, vel):

        cornor_list = [] # initial cornor list which contains the cornors

        node_r = node.row # row of the input node
        node_c = node.col # column of the input node

        vec_2_goal = np.array([node_r-self.goal[0],node_c-self.goal[1]])
        
        if np.linalg.norm(vec_2_goal) != 0:
            dir_2_goal = vec_2_goal / np.linalg.norm(vec_2_goal)   # unit vector heading to goal

        else:
            dir_2_goal = np.zeros(2)

        weight_dir = dir_2_goal + vel # prefered direction vector for motion primitive 
        

        '''
        step 1 : search direction 
        
        steo 2 : jump search, append the cornor nodes in cornor_list if detected during search

        step 3 : for nodes in cornor_list, choose one with the lowest cost, and run search() 
                 than, pop the node if the search() returns False

        step 4 : run search() with diagonal adjacent nodes, and if it returns True, return True
        '''

        ''' search for cornors and goal '''
        # print("UR quad")
        if self.UR_search(node, cornor_list):

            return True

        # print("DR quad")
        if self.DR_search(node, cornor_list):

            return True

        # print("DL quad")
        if self.DL_search(node, cornor_list):

            return True

        # print("UL quad")
        if self.UL_search(node, cornor_list):

            return True

        ''' repeat for cornor with minimal cost if no goal detected '''

        while len(cornor_list) != 0:
            
            min_cost_cornor = cornor_list[0]
            min_cost_cornor_idx = 0

            # find cornor with lowest cost
            for idx, cornor in enumerate(cornor_list):


                if cornor.cost(self.start, self.goal, weight_dir,self.vel_size) < min_cost_cornor.cost(self.start, self.goal, weight_dir,self.vel_size):
                    
                    min_cost_cornor = cornor
                    min_cost_cornor_idx = idx


            del cornor_list[min_cost_cornor_idx]



            ''' run search function recursively for node with the minimal cost '''

            # classufy the direction of the cornor
            if min_cost_cornor.cornor_dir == "UL":

                new_search_point_node = Node(min_cost_cornor.row,min_cost_cornor.col-1,min_cost_cornor)
                # print("search UL Cornor")

                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True


            if min_cost_cornor.cornor_dir == "UR":

                new_search_point_node = Node(min_cost_cornor.row,min_cost_cornor.col+1,min_cost_cornor)
                # print("search UR Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "RU":

                new_search_point_node = Node(min_cost_cornor.row-1,min_cost_cornor.col,min_cost_cornor)
                # print("search RU Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "RD":

                new_search_point_node = Node(min_cost_cornor.row+1,min_cost_cornor.col,min_cost_cornor)
                # print("search RD Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "DL":

                new_search_point_node = Node(min_cost_cornor.row,min_cost_cornor.col-1,min_cost_cornor)
                # print("search DL Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "DR":

                new_search_point_node = Node(min_cost_cornor.row,min_cost_cornor.col+1,min_cost_cornor)
                # print("search DR Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "LU":

                new_search_point_node = Node(min_cost_cornor.row-1,min_cost_cornor.col,min_cost_cornor)
                # print("search LU Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True
                

            if min_cost_cornor.cornor_dir == "LD":

                new_search_point_node = Node(min_cost_cornor.row+1,min_cost_cornor.col,min_cost_cornor)
                # print("search LD Cornor")
                
                # if the goal is in new search direction,
                # append the node in the closed list
                if self.search(new_search_point_node,vel/2):

                    return True





    def run(self):

        start_node = Node(self.start[0],self.start[1],None) 
        result = self.search(start_node,self.vel_vec)

        center = int(len(self.map)/2)

        if result:


            waypoints = np.array([[0],[0]])

            if len(self.closedlist) == 0:
                node = None
            else:
                node = self.closedlist[0]

            while node != None:

                if node.cornor_dir != None:

                    
                    # convert row-col to x-y for cornor
                    waypoint = np.array([[(center) - node.row],
                                         [node.col - (center)]])
                
                    waypoints = np.hstack((waypoint,waypoints))


                    # convert row-col to x-y for parent of the cornor
                    waypoint = np.array([[(center) - node.Parent.row],
                                         [node.Parent.col - (center)]])

                    if not ((node.Parent.col-center)**2+(node.Parent.col-center)**2) < 20:
                        waypoints = np.hstack((waypoint,waypoints))

    
                node = node.Parent


            waypoints = waypoints[:,:-1]

            if len(waypoints[0]) == 0:

                # print("No obstacle on my way         ",end="\r")
                
                return np.array([])

            else:

                # print(f"obstacle detected         ")

                return waypoints

        else:
            # print("No way to goal !         ",end="\r")

            return np.array([])