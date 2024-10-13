#!/usr/bin/env python3
# Github link : https://github.com/gautamsrn/ENPM661-Project-3.2
#ENPM661 : Project 3 - Phase:2 (part-1)
#**************Team*******************#
#Piyush Goenka (120189500) - pgoenka
#Gautam Sreenarayanan Nair(119543092) - gautamsn
# ***************************************************************************
# Importing neccessary libraries
import numpy as np
import cv2
import math 
from queue import PriorityQueue as PQ
import time

# Class for planning
class PathPlanning:

    def __init__(self):

        try:
            # attributes for the class.
            self.width =600
            self.height = 200
            self.lines = {}
            self.curve=[]
            self.start_point = (40,100)
            self.goal_point = (550,175) 
            self.start_point_orientation = 180
            self.clearance = 1
            self.robot_radius = 22   
            self.wheel_radius = 3.3  
            self.wheel_distance = 28.7 
            self.weight = 1.8
            self.final_node = None
            self.rpm1 =10
            self.rpm2 =50
            self.fixed_time = None
            self.optimal_path= None
            self.user_input()
            
            
            self.node_queue = PQ()
            self.node_queue.put((0, (self.start_point,self.start_point_orientation)))
            self.current_edge_cost = {self.start_point: 0}
            self.current_total_cost = {self.start_point: 0}
            self.parent = {self.start_point: self.start_point}
            self.visited_list= []
            self.path_end_list=[]
            self.coords_angle_dict={}

            self.goal_threshold = 0.5* (self.robot_radius)
            self.astar()
            self.animate()
    
        except:
            print("Path Not found! Try changing input variables")
    
     # function to make map
    def load_map(self):
        # Making the map
        height = 200
        width = 600
        self.map_data = np.zeros((height, width), dtype=int)
        self.normal_map = np.zeros((height, width), dtype=int)
        # Map_plot for plotting purpose - white background
        self.map = np.full((height, width, 3), 255, dtype=np.uint8)

        print("Creating Map with obstacles...")
        
        # Assigning the value 1 to obstacle positions.
        for y in range(height):
            for x in range(width):
                thickness = self.clearance + self.robot_radius
                circle = ((x-420)*(x-420) + (y-120)*(y-120)) <= (60+thickness)*(60+thickness)
                # Obstacle check using half plane equation
                if ((150 - thickness) <= x <= (175+thickness) and (100-thickness) <= y <= 200) or \
                ((250-thickness) <= x <= (275+thickness) and 0 <= y <= (100+thickness)) or\
                                (0 <= x <= thickness) or \
                ((600-thickness) <= x <= 600) or \
                (0 <= y <= thickness) or \
                ((200-thickness) <= y <= 200) or \
                    circle:
                    self.map_data[y, x] = 1   
        # Assigning color to obstacles
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] == 1:
                    self.map[height - y - 1, x] = (255,90,90)   #color for non free space and shifting origin to bottom left

        for y in range(height):
            for x in range(width):
                thickness = 0
                circle = ((x-420)*(x-420) + (y-120)*(y-120)) <= (60+thickness)*(60+thickness)
                # Obstacle check using half plane equation
                if ((150 - thickness) <= x <= (175+thickness) and (100-thickness) <= y <= 200) or \
                ((250-thickness) <= x <= (275+thickness) and 0 <= y <= (100+thickness)) or\
                                                (0 <= x <= thickness) or \
                ((600-thickness) <= x <= 600) or \
                (0 <= y <= thickness) or \
                ((200-thickness) <= y <= 200) or \
                    circle:
                    self.map_data[y, x] = 2         
        print("Map Created")           
                    
        # Assigning color to obstacles
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] == 2:
                    self.map[height - y - 1, x] = (90,90,90)   #color for non free space and shifting origin to bottom left

 
    # This function is used to convert co-ordinates to cv2 co-ordinates for retrival of pixel values with respect to origin as we need.
    def normal_coords_to_cv2(self,x_normal, y_normal):
        y_cv2 = (self.height -1)  - y_normal
        x_cv2 = x_normal
        return x_cv2, y_cv2

    # This function prompts user for input
    def user_input(self):

        self.start_point=None
        self.goal_point=None
        self.start_point_orientation = None

        while True:

            user_input = input("Please enter the clearance (in mm): ")  # check unit here
            clearance = int(user_input)
            print("Clearance recorded!")
            break  

        self.clearance = int(clearance/10) # converting to cm

        self.load_map()
     

        while True:

            user_input = input("Please input the start point coordinates [x(in mm) y(in mm) theta] separated by a space: ")
            coords = user_input.split()
            if len(coords)!=3:
                print ("Invaid point! Try again...")
                
            else:
                coords[0]= int(coords[0])
                coords[1]=int(coords[1])
                coords[2] = int(coords[2])
                start_x, start_y = (coords[0], coords[1])
                start_x = start_x/10
                start_y = start_y/10
                if int(start_x)<0 or int(start_x)>=600 or int(start_y)<0 or int(start_y)>=200 :
                    print ("Invaid point! Try again...")

                elif self.map_data[int(start_y)][int(start_x)]!=0:
                    print ("Invaid point! Try again...")
                
                else:
                    start_point=(int(start_x),int(start_y))
                    start_point_orientation = coords[2]%360
                    print("Start point details recorded!")
                    break

        while True:

            user_input = input("Please input the goal point coordinates [x y] separated by a space: ")
            coords = user_input.split()
            if len(coords)!=2:
                print ("Invaid point! Try again...")
            else:     
                coords[0]= int(coords[0])
                coords[1]=int(coords[1])
                goal_x, goal_y = (coords[0], coords[1])
                goal_x=goal_x/10
                goal_y=goal_y/10
                if int(goal_x)<0 or int(goal_x)>=600 or int(goal_y)<0 or int(goal_y)>=200 :
                    print ("Invaid point! Try again...")
                elif self.map_data[int(goal_y)][int(goal_x)]!=0:
                    print ("Invaid point! Try again...")
                else:
                    goal_point=(int(goal_x),int(goal_y))
                    print("Goal point details recorded!")
                    break   

        while True:

            user_input = input("Please input the two RPMS seperated by space: ")
            rpms = user_input.split()
            if len(rpms)!=2:
                print ("Invaid point! Try again...")
            else:     
                rpms[0]= int(rpms[0])
                rpms[1]=int(rpms[1])
                rpm1, rpm2 = (rpms[0], rpms[1])
                rpm_detail=(rpm1,rpm2)
                print("RPM details recorded!")
                break   

        print(f"Start point--> {int(start_x*10)}, {int(start_y*10)} {start_point_orientation}")
        print(f"Goal point--> {int(goal_x*10)}, {int(goal_y*10)}")
        print(f"Clearance --> {clearance}")
        print(f"RPM values --> {rpm_detail}")
 
       
        self.start_point = start_point
        self.goal_point = goal_point 
        self.start_point_orientation = start_point_orientation
        self.rpm1 = rpm1
        self.rpm2 =rpm2

        
        print ("Computing path...") 

    # Function that handles the actions
    def action_set(self, node):
        next_node = []
        x_in = node[0][0]
        y_in = node[0][1]
        theta_in = node[1]
        # looping through action set
        for action in [[0, self.rpm1], [self.rpm1, 0], [self.rpm1, self.rpm1],
                    [0, self.rpm2], [self.rpm2, 0], [self.rpm2, self.rpm2],
                    [self.rpm1, self.rpm2], [self.rpm2, self.rpm1]]:
            UL = action[0]
            UR = action[1]
            next_node_details = self.cost(x_in, y_in, theta_in, UL, UR)
            # Check if the resulting position is within the map and not obstructed
            x, y = next_node_details[0], next_node_details[1]
            is_curve_free = False
            if 0 < x < 600 and 0 < y < 200 and self.map_data[y][x] == 0 :
                is_curve_free=True
                for points in self.curve.copy():
                    xp, yp = points
                    if not (0 < xp < 600 and 0 < yp < 200 and self.map_data[yp][xp] == 0) :
                        is_curve_free = False
                        break
            if is_curve_free ==True:
                self.lines[(x,y,x_in,y_in)]= self.curve.copy()
                next_node.append(next_node_details)
                
        return next_node
    
    # Function to compute the curvature time
    def compute_curvature_time(self, theta = 90):
        speed = 2*math.pi*self.rpm2/60
        if self.rpm1>= self.rpm2:
            speed = 2*math.pi*self.rpm1/60
        theta = theta *3.14/180
        self.fixed_time = (theta * self.wheel_distance)/(speed*self.wheel_radius)
    # Function for cost calculation
    def cost(self,Xi,Yi,Thetai,UL,UR):
        t = 0
        r =self.wheel_radius
        L = self.wheel_distance
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
        UL = 2*math.pi*UL/60
        UR = 2*math.pi*UR/60
        D=0
        self.curve =[]
        dt = self.fixed_time/100
        
        # looping for integration
        while t< self.fixed_time:
            t = t + dt
            # finding small change in x,y and theta
            Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            Xn +=Delta_Xn
            Yn+=Delta_Yn
            D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
            # appending the points for plotting
            self.curve.append((int (Xn), int(Yn)))
        Thetan = (180 * (Thetan) / 3.14)
        return int(Xn), int(Yn), int(Thetan), round(D,3)
    

    # this function generates the animation
    def animate(self):
        start_x, start_y = self.normal_coords_to_cv2(self.start_point[0], self.start_point[1])
        goal_x, goal_y = self.normal_coords_to_cv2(self.goal_point[0], self.goal_point[1])
        node_incr = 0
        # draw goal point
        cv2.circle(self.map, (goal_x, goal_y), 6, (50, 50, 255), -1) 
        # draw the threshold circle
        cv2.circle(self.map, (goal_x, goal_y),int(self.goal_threshold), (130,130,130), 1) 
        # draw start point
        cv2.circle(self.map, (start_x, start_y), 6, (50, 255, 50), -1)  
        
        # To display the node visit sequence
        for visited_node in self.visited_list:
            node_incr += 1
            xn,yn= self.normal_coords_to_cv2(visited_node[0],visited_node[1])
            
            xv, yv = visited_node
            xp, yp = self.parent[visited_node]
            curve = self.lines[(xv,yv,xp,yp)]
            for point in curve:
                px, py = point
                ppx, ppy = self.normal_coords_to_cv2(px, py)
                cv2.circle(self.map, (ppx, ppy), 1, (150,150,150), -1) 
            # To speed up the animation we update frame every 10 nodes
            # if node_incr % 25 == 0:
            cv2.imshow("Map", self.map)
            cv2.waitKey(1)
        
        # To display the path
        for node in self.optimal_path :
            
            xn,yn= self.normal_coords_to_cv2(node[0],node[1])
            self.map[yn, xn] = (150, 150, 150)
            xn,yn = self.normal_coords_to_cv2(node[0], node[1])
            xv, yv = node
            xp, yp = self.parent[node]
            if node != self.start_point:
                curve = self.lines[(xv,yv,xp,yp)]
                for point in curve:
                    px, py = point
                    ppx, ppy = self.normal_coords_to_cv2(px, py)
                    cv2.circle(self.map, (ppx, ppy), 1, (0,0,0), -1) 
            cv2.imshow("Map", self.map)
            cv2.waitKey(1)
            time.sleep(0.05)

        cv2.imshow("Map", self.map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
                
    # Function to calculate the euclidean distance between 2 nodes.
    def euclidean_distance(self, node1, node2):
        d = math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)
        return round(d,4)
    

    # Backtracking function
    def backtracking(self,final_node):
        # Initializing path list with goal points
        path = []
        # Current node is (node_index, parent_node_index, coordinates)
        current_node = final_node
        path.append (current_node)
        # Looping as long as we reach initial point
        while (current_node) != (self.start_point):
            parent_node=self.parent[current_node]
            path.append(parent_node)
            current_node = parent_node
        # Reversing the path
        path.reverse()
        return path

    # This function runs the astar algorithm
    def astar(self):
        self.compute_curvature_time()
        # Loop until queue has no nodes
        while not self.node_queue.empty():
            # taking the lowest cost node.
            present_total_cost, node = self.node_queue.get()
            # taking node coordinates
            node_coordinates = node[0]
            # break if goal threshold and orientation reached
            if (self.euclidean_distance(node_coordinates, self.goal_point) <= self.goal_threshold):
                print ("Goal reached!")
                self.final_node = node_coordinates
                # call backtracking function
                self.optimal_path = self.backtracking(self.final_node)
                break
            # Finding next nodes through action set
            adjacent_nodes = self.action_set(node)
            # Looping through newly found nodes
            for adjacent_node in adjacent_nodes:
                # Taking coordinates of new nodes.
                adjacent_node_coords = adjacent_node[:2]
                # Calculation the added cost.
                added_edge_cost =  adjacent_node[3]
                # Calculating cost to come
                updated_edge_cost = self.current_edge_cost[node_coordinates] + added_edge_cost
                # Calculating the cost to go 
                heuristic_cost = self.euclidean_distance(adjacent_node_coords,self.goal_point)*self.weight
                # Calculating the total cost
                total_cost = heuristic_cost + updated_edge_cost
                # add/update visited nodes
                if (adjacent_node_coords not in self.visited_list ) or( total_cost < self.current_total_cost.get(adjacent_node_coords, float('inf'))):
                    # updating the cost
                    self.current_edge_cost[adjacent_node_coords] = updated_edge_cost
                    self.current_total_cost[adjacent_node_coords]= total_cost
                    # Calculating the orientation from 0 to 360.
                    orientation = ((360+adjacent_node[2])%360)
                    # Putting the nodes in queue
                    self.node_queue.put((total_cost,(adjacent_node_coords,orientation)))
                    # Updating parent dictionary for backtracking
                    self.parent[adjacent_node_coords] = node_coordinates
                    # adding node to visited list 
                    self.visited_list.append(adjacent_node[:2])
                   
if __name__ == "__main__":
    PathPlanning()
    