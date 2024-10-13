#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from turtlebot3_project3.msg import UserInput 
import math 
import numpy as np
import cv2
from queue import PriorityQueue as PQ
import rclpy.time
from rclpy.clock import Clock


# Class for planning
class PathPlanning:

    def __init__(self):

        try:
            self.width =610
            self.height = 300
            self.lines = {}
            self.velocities={}
            self.curve=[]
            self.vel=[]
            self.start_point = (1,150) 
            self.goal_point=(605,210)
            self.start_point_orientation = 0
            self.clearance = 2
            self.robot_radius = 22   # done
            self.wheel_radius = 3.3  # done
            self.wheel_distance = 28.7  # done
            self.weight = 3
            self.wall_clearance = 10
            self.total_wall_clearance = self.wall_clearance + self.robot_radius
            self.final_node = None
            self.rpm1 =75
            self.rpm2 =75
            self.fixed_time = None
            self.is_goal_reached=False
            self.optimal_path= None
                  
            self.node_queue = PQ()
            self.node_queue.put((0, (self.start_point,self.start_point_orientation)))
            self.current_edge_cost = {self.start_point: 0}
            self.current_total_cost = {self.start_point: 0}
            self.parent = {self.start_point: self.start_point}
            self.visited_list= []
            self.path_end_list=[]
            self.coords_angle_dict={}

            self.goal_threshold = 0.5* (self.robot_radius)
            self.load_map()

    
        except:
            print("Path Not found! Try changing input variables")
    
     # function to make map
    def load_map(self):
        # Making the map
        height = 300
        width = 605
        self.map_data = np.zeros((height, width), dtype=int)
        # Map_plot for plotting purpose - white background
        self.map = np.full((height, width, 3), 255, dtype=np.uint8) 
        self.total_clearance
        #obstacles
        for i in range(180+self.total_clearance):
            self.map_data [i][(150-self.total_clearance):(152+self.total_clearance)][:]=1
            self.map_data [i][(450-self.total_clearance):(452+self.total_clearance)][:]=1
        for i in range((120-self.total_clearance),300):
            self.map_data [i][(300-self.total_clearance):(302+self.total_clearance)][:]=1

        for i in range(230-self.total_clearance,260+self.total_clearance):
            self.map_data [i][(60-self.total_clearance):(90+self.total_clearance)][:]=1
            self.map_data [i][(210-self.total_clearance):(240+self.total_clearance)][:]=1
            self.map_data [i][(360-self.total_clearance):(390+self.total_clearance)][:]=1
            self.map_data [i][(510-self.total_clearance):(540+self.total_clearance)][:]=1

        for i in range(60-self.total_clearance,90+self.total_clearance):
            self.map_data [i][(210-self.total_clearance):(240+self.total_clearance)][:]=1
            self.map_data [i][(360-self.total_clearance):(390+self.total_clearance)][:]=1
            self.map_data [i][(510-self.total_clearance):(540+self.total_clearance)][:]=1
        
        for i in range(self.total_wall_clearance):
            self.map_data [i][:][:]=1
            
        for i in range(300-self.total_wall_clearance,300):
            self.map_data [i][:][:]=1
            
        # for i in range(0,300):
        #     self.map_data [i][(600):(601)][:]=1


            
         # Assigning color to obstacles
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] == 1:
                    self.map[y, x] = (255,90,90)   #color for non free space and shifting origin to bottom left                    


    print("map created")             

    # Function to return the computed path data
    def return_path_data(self):

        velocity_list =[]
        position_list =[]
        velocity_list.append((0.0,0.0,0.0))
        for node in self.optimal_path :
            
            xv, yv = node
            xp, yp = self.parent[node]
            if node != self.start_point:
                velocity = self.velocities[(xv,yv,xp,yp)]
                curve= self.lines[(xv,yv,xp,yp)]
                for vel in velocity:
                    velocity_list.append(vel)
                for point in curve:
                    position_list.append(point)

        
        dict = {}
        dict["update_rate"]= self.fixed_time/100
        dict["velocity"] = velocity_list
        dict["position"] = position_list

        return dict

 
    # This function is used to convert co-ordinates to cv2 co-ordinates for retrival of pixel values with respect to origin as we need.
    def normal_coords_to_cv2(self,x_normal, y_normal):
        y_cv2 = (self.height -1)  - y_normal
        x_cv2 = x_normal
        return x_cv2, y_cv2

    # Function that handles the actions
    def action_set(self, node):
        next_node = []
        x_in = node[0][0]
        y_in = node[0][1]
        theta_in = node[1]
        # looping through action set
        for action in [[self.rpm1, self.rpm1],
                     [self.rpm2, self.rpm2],
                     [self.rpm2/2, self.rpm1],[self.rpm2, self.rpm1/2],[self.rpm2, self.rpm1],[self.rpm2*0.75, self.rpm1],[self.rpm2, self.rpm1*0.75],[self.rpm2*0.25, self.rpm1],[self.rpm2, self.rpm1*0.25],[self.rpm2*0.1, self.rpm1],[self.rpm2, self.rpm1*0.1]]:
            UL = action[0]
            UR = action[1]
            next_node_details = self.cost(x_in, y_in, theta_in, UL, UR)
            # Check if the resulting position is within the map and not obstructed
            x, y = next_node_details[0], next_node_details[1]
            is_curve_free = False
            if 0 < x < 605 and 0 < y < 300 and self.map_data[y][x] == 0 :
                is_curve_free=True
                for points in self.curve.copy():
                    xp, yp = points
                    if not (0 < xp < 605 and 0 < yp < 300 and self.map_data[yp][xp] == 0) :
                        is_curve_free = False
                        break
            if is_curve_free ==True:
                self.lines[(x,y,x_in,y_in)]= self.curve.copy()
                next_node.append(next_node_details)
                
        return next_node
    # Here we compute the amount of time for which a curve needs to be iterated upon
    def compute_curvature_time(self, theta =90):
        speed = self.rpm2*2*3.14/60
        if self.rpm1>= self.rpm2:
            speed = self.rpm1*2*3.14/60
        theta = theta *3.14/180
        self.fixed_time = (theta * self.wheel_distance)/(speed*self.wheel_radius)

    # cost function
    def cost(self,Xi,Yi,Thetai,UL,UR):
        t = 0
        r =self.wheel_radius
        L = self.wheel_distance
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
        UL = UL*2*3.14/60
        UR = UR*2*3.14/60

        # Xi, Yi,Thetai: Input point's coordinates
        # Xn, Yn, Thetan: End point coordintes
        D=0
        self.curve =[]
        self.vel =[]
        dt = self.fixed_time/100
        
        while t<self.fixed_time:
            t = t + dt

            Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Delta_Theta = (r / L) * (UR - UL) * dt
            Thetan += Delta_Theta
            Xn +=Delta_Xn
            Yn+=Delta_Yn
            D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
            self.curve.append((int (Xn), int(Yn)))
            Vel_Xn = Delta_Xn/(dt*100)
            Vel_Yn = Delta_Yn/(dt*100)
            Vel_theta = Delta_Theta/dt
            self.vel.append((Vel_Xn, Vel_Yn,Vel_theta))   # m/s , m/s, rad
        Thetan = (180 * (Thetan) / 3.14)
      
        return int(Xn), int(Yn), int(Thetan), round(D,3)
    

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
        
        try:
            self.compute_curvature_time()

            infinite = float('inf')
            # Loop until queue has nodes
            while not self.node_queue.empty():
                if self.is_goal_reached == True:
                    break

                # taking the lowest cost node.
                _, node = self.node_queue.get()
                
                node_coordinates = node[0]

                # break if goal threshold and orientation reached
                if (self.euclidean_distance(node_coordinates, self.goal_point) <= self.goal_threshold):
                    self.final_node = node_coordinates
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
                    # Calculating the cost to go (weight is one)
                    heuristic_cost = self.euclidean_distance(adjacent_node_coords,self.goal_point)*self.weight
                    # Calculating the total cost
                    total_cost = heuristic_cost + updated_edge_cost

                    # add/update visited nodes

                    if (adjacent_node_coords not in self.visited_list ) or( total_cost < self.current_total_cost.get(adjacent_node_coords, float('inf'))):
                        # updating the cost
                        self.current_edge_cost[adjacent_node_coords] = updated_edge_cost
                        self.current_total_cost[adjacent_node_coords]= total_cost
                        # Calculating the orientation
                       
                        orientation = ((360+adjacent_node[2])%360)
                      
                        # Putting the nodes in queue

                        self.node_queue.put((total_cost,(adjacent_node_coords,orientation)))
                        # Updating parent dictionary for backtracking
                        self.parent[adjacent_node_coords] = node_coordinates
                        # adding node to visited list 
                        self.visited_list.append(adjacent_node[:2])  
                    
            data = self.return_path_data()


        except:
            print("Path Not found! Try changing input variables")
            data = None

        finally:  

            return data
        

# Open loop Controller Node 
class Controller(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.planning_done_flag = False
        self.user_input_sub = self.create_subscription(UserInput, '/user_input', self.user_input_callback, 10)
        self.planner = PathPlanning()
        self.counter = 0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.clock = Clock()

    def user_input_callback(self, msg):
        # Extract start and goal points from the received message
        self.get_logger().info("User input received : processing the input .....")
        goal_x = int(msg.goal_x)
        goal_y = int(msg.goal_y)
        goal_x_cm = int(goal_x/10)
        goal_y_cm = int(goal_y/10)

        # Check if the start and goal points are valid
        if self.planner.map_data[goal_y_cm][goal_x_cm] == 0:
            if goal_x_cm > 0 and goal_x_cm < 600 and goal_y_cm >0 and goal_y_cm < 200:
                self.planner.goal_point = (goal_x_cm,goal_y_cm)
                self.get_logger().info("Inputs accepted")
                self.run_astar()
        else:
            self.get_logger().info("Invalid points! Try again...")

    def run_astar(self):
        self.get_logger().info("Planning path")
        self.data = self.planner.astar()
        self.get_logger().info("Path Planning completed")
        self.veloctiy_list = self.data["velocity"]
        self.rate = self.data["update_rate"]
        self.planning_done_flag = True
        self.timer_ = self.create_timer(self.rate, self.publish_cmd_vel) # 10 Hz
        
        
    def publish_cmd_vel(self):

        if self.planning_done_flag and self.counter<len(self.veloctiy_list):
            self.get_logger().info("counter: %s" % str(self.counter))
            self.get_logger().info("rate: %s" % str(self.rate))

            vel = self.veloctiy_list[self.counter]
            vel_x , vel_y, vel_theta = vel
            linear_speed = math.sqrt(vel_x**2 + vel_y**2)
            angular_speed = vel_theta
            msg = Twist()
            # Set linear velocity
            msg.linear.x = linear_speed  # Example value
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            # Set angular velocity
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = angular_speed # Example value
            self.publisher_.publish(msg)
            self.get_logger().info(f'My log message ')
        else:
            self.get_logger().info(f'none')
            self.get_logger().info("counter: %s" % str(self.counter))
            msg = Twist()
            self.publisher_.publish(msg)
        self.counter +=1
