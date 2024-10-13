#!/usr/bin/env python3
# Github link : https://github.com/gautamsrn/ENPM661-Project-3.2
# ENPM661 : Project 3 - Phase:2 (part-1)
# **************Team*******************#
# Piyush Goenka (120189500) - pgoenka
# Gautam Sreenarayanan Nair(119543092) - gautamsn
# ***************************************************************************
# Importing neccessary libraries
import numpy as np
import cv2
import math
from queue import PriorityQueue as PQ
import time
import csv
# Class for planning


class PathPlanning:

    def __init__(self, start_point=(0, 150), start_point_orientation=0,goal_point=(649, 200)):

        try:
            # attributes for the class.
            self.width = 660
            self.height = 300
            self.lines = {}
            self.curve = []
            self.node_orientation_dict={}
            self.vel = []
            self.velocities = {}
            self.start_point = start_point
            self.goal_point = goal_point
            self.start_point_orientation = start_point_orientation
            self.clearance_x = 5
            self.clearance_y = 7
            self.robot_radius = 22
            self.total_clearance_x = self.clearance_x + self.robot_radius
            self.total_clearance_y = self.clearance_y + self.robot_radius
            self.wheel_radius = 3.3
            self.wheel_distance = 28.7
            self.weight = 3
            self.final_node = None
            self.wall_clearance = 55
            self.line_threshold = 10
            self.total_wall_clearance = self.wall_clearance + self.robot_radius
            self.rpm1 = 75
            self.rpm2 = 75/2.0
            self.curvature_angle =45
            self.fixed_time = None
            self.optimal_path = None
            # self.user_input()

            self.load_map()
            self.node_queue = PQ()
            self.node_queue.put(
                (0, (self.start_point, self.start_point_orientation)))
            self.current_edge_cost = {self.start_point: 0}
            self.current_total_cost = {self.start_point: 0}
            self.parent = {self.start_point: self.start_point}
            self.visited_list = []
            self.path_end_list = []
            self.coords_angle_dict = {}

            self.goal_threshold = 0.5 * (self.robot_radius)
            self.goal_threshold=5

        except:
            print("Path Not found! Try changing input variables")



    def goal_checker(self, node):
        xn = node[0]
        yn = node[1]

        xg, yg = self.goal_point
        if (xn+self.line_threshold >= xg):
            return True
        else:
            return False


     # function to make map
    def load_map(self):
        
        # Initialize map data and map plot
        self.map_data = np.zeros((self.height, self.width), dtype=int)
        self.map = np.full((self.height, self.width, 3), 255, dtype=np.uint8)  # White background
        
        # Define obstacle positions
        for y_val in range(180 + self.total_clearance_y):
            self.map_data[y_val, 150 - self.total_clearance_x : 152 + self.total_clearance_x] = 1
            self.map_data[y_val, 450 - self.total_clearance_x : 452 + self.total_clearance_x] = 1
        
        for y_val in range(120 - self.total_clearance_y, 300):
            self.map_data[y_val, 300 - self.total_clearance_x : 302 + self.total_clearance_x] = 1
        
        for y_val in range(230 - self.total_clearance_y, 260 + self.total_clearance_y):
            self.map_data[y_val, 60 - self.total_clearance_x : 90 + self.total_clearance_x] = 1
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 1
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 1
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 1
        
        for y_val in range(60 - self.total_clearance_y, 90 + self.total_clearance_y):
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 1
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 1
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 1
        
        self.add_clearance_color()
        
        # Add wall clearance
        for y_val in range(self.total_wall_clearance):
            self.map_data[y_val, :] = 1
        
        for y_val in range(300 - self.total_wall_clearance, 300):
            self.map_data[y_val, :] = 1
        
        # Assign colors to obstacles
        for y in range(self.height):
            for x in range(self.width):
                if self.map_data[y, x] == 1:
                    self.map[y, x] = (255, 90, 90)  # Non-free space color
                elif self.map_data[y, x] == 2:
                    self.map[y, x] = (200, 200, 0)
        
        # Assign goal line
        xg,yg=self.goal_point
        for y in range(self.height):
            x =xg - self.line_threshold
            self.map[y, x] = (0, 255, 0)  # green line

        print("Map created")


    def add_clearance_color(self):
        # Define obstacle positions
        for y_val in range(180+self.robot_radius,180 +self.robot_radius+ self.clearance_y ):
            self.map_data[y_val, 150 - self.total_clearance_x : 152 + self.total_clearance_x] = 2
            self.map_data[y_val, 450 - self.total_clearance_x : 452 + self.total_clearance_x] = 2
        
        for y_val in range(120-self.robot_radius - self.clearance_y, 120-self.robot_radius):
            self.map_data[y_val, 300 - self.total_clearance_x : 302 + self.total_clearance_x] = 2
        
        for y_val in range(230-self.robot_radius - self.clearance_y, 230-self.robot_radius):
            self.map_data[y_val, 60 - self.total_clearance_x : 90 + self.total_clearance_x] = 2
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 2
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 2
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 2
            
        for y_val in range(260+self.robot_radius, 260 +self.robot_radius+ self.clearance_y):
            self.map_data[y_val, 60 - self.total_clearance_x : 90 + self.total_clearance_x] = 2
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 2
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 2
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 2
        
        for y_val in range(60 -self.robot_radius-self.clearance_y, 60-self.robot_radius):
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 2
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 2
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 2
            
        for y_val in range(90+self.robot_radius, 90+self.robot_radius+ self.clearance_y):
            self.map_data[y_val, 210 - self.total_clearance_x : 240 + self.total_clearance_x] = 2
            self.map_data[y_val, 360 - self.total_clearance_x : 390 + self.total_clearance_x] = 2
            self.map_data[y_val, 510 - self.total_clearance_x : 540 + self.total_clearance_x] = 2
        
#----------------------------------------------------------------------------------------- left clerance
        for y_val in range(180 + self.total_clearance_y):
            self.map_data[y_val, 150 - self.robot_radius -self.clearance_x : 150 - self.robot_radius] = 2
            self.map_data[y_val, 450 - self.robot_radius - self.clearance_x : 450 - self.robot_radius] = 2
        
        for y_val in range(120 - self.total_clearance_y, 300):
            self.map_data[y_val, 300 - self.robot_radius -self.clearance_x : 300 - self.robot_radius] = 2
        
        for y_val in range(230 - self.total_clearance_y, 260 + self.total_clearance_y):
            self.map_data[y_val, 60 - self.robot_radius-self.clearance_x  : 60 - self.robot_radius] = 2
            self.map_data[y_val, 210 - self.robot_radius-self.clearance_x  : 210 - self.robot_radius] = 2
            self.map_data[y_val, 360 - self.robot_radius-self.clearance_x  : 360 - self.robot_radius] = 2
            self.map_data[y_val, 510 - self.robot_radius-self.clearance_x  : 510 - self.robot_radius] = 2
            
        for y_val in range(60 - self.total_clearance_y, 90 + self.total_clearance_y):
            self.map_data[y_val, 210 - self.robot_radius-self.clearance_x  : 210 - self.robot_radius] = 2
            self.map_data[y_val, 360 - self.robot_radius -self.clearance_x : 360 - self.robot_radius] = 2
            self.map_data[y_val, 510 - self.robot_radius-self.clearance_x  : 510 - self.robot_radius] = 2            
#----------------------------------------------------------------------------------------- right clerance
        for y_val in range(180 + self.total_clearance_y):
            self.map_data[y_val, 152 + self.robot_radius : 152 + self.robot_radius+self.clearance_x ] = 2
            self.map_data[y_val, 452 + self.robot_radius : 452 + self.robot_radius+self.clearance_x ] = 2
        
        for y_val in range(120 - self.total_clearance_y, 300):
            self.map_data[y_val, 302 + self.robot_radius : 302 + self.robot_radius+self.clearance_x] = 2
        
        for y_val in range(230 - self.total_clearance_y, 260 + self.total_clearance_y):
            self.map_data[y_val, 90 + self.robot_radius : 90 + self.robot_radius +self.clearance_x] = 2
            self.map_data[y_val, 240 + self.robot_radius : 240 + self.robot_radius+self.clearance_x] = 2
            self.map_data[y_val, 390 + self.robot_radius : 390 + self.robot_radius+self.clearance_x] = 2
            self.map_data[y_val, 540 + self.robot_radius : 540 + self.robot_radius+self.clearance_x] = 2
        
        for y_val in range(60 - self.total_clearance_y, 90 + self.total_clearance_y):
            self.map_data[y_val, 240 + self.robot_radius : 240 + self.robot_radius+self.clearance_x] = 2
            self.map_data[y_val, 390 + self.robot_radius : 390 + self.robot_radius+self.clearance_x] = 2
            self.map_data[y_val, 540 + self.robot_radius : 540 + self.robot_radius+self.clearance_x] = 2
        


    # Function to return the computed path data

    def return_path_data(self):

        velocity_list = []
        position_list = []

        for node in self.optimal_path:

            xv, yv = node
            xp, yp = self.parent[node]
            if node != self.start_point:
                velocity = self.velocities[(xv, yv, xp, yp)]
                curve = self.lines[(xv, yv, xp, yp)]
                for vel in velocity:
                    vel_x, vel_y, vel_t = vel
                    vel_linear = round(math.sqrt(vel_x**2 + vel_y**2),5)
                    velocity_list.append((vel_linear,vel_t ))
                for point in curve:
                    position_list.append(point)

        velocity_list.append((0.0,0.0))
        velocity_list.append((0.0,0.0))

        velocity_list = self.adjust_velocity(velocity_list)
        

        dict = {}
        dict["update_rate"] = self.fixed_time/100
        dict["velocity"] = velocity_list
        dict["position"] = position_list

        final_point = position_list[-1]
        fx, fy = final_point
        dict["goal_data"] = (fx, fy, self.node_orientation_dict[final_point])
        

        return dict

    # # This function is used to convert co-ordinates to cv2 co-ordinates for retrival of pixel values with respect to origin as we need.
    # def normal_coords_to_cv2(self,x_normal, y_normal):
    #     y_cv2 =  y_normal
    #     x_cv2 = x_normal
    #     return x_cv2, y_cv2

    # Function that handles the actions

    def adjust_velocity(self, input_tuple):
        my_list = list(input_tuple)

        new_vel_list = []

        length = len(my_list) -1
        
        pointer = 0
        while (pointer < length):

            lin_v, ang_v = my_list[pointer]

            if (0.125<=lin_v <= 0.13) and abs(ang_v) ==0.0:

                lin_v_next, ang_v_next = my_list[pointer+1]

                if (0.125<=lin_v_next <= 0.13) and abs(ang_v_next) ==0.0:

                    new_vel_list.append((0.25918, 0.0))
                    pointer+=2

                else:
                    new_vel_list.append((lin_v, ang_v))
                    pointer+=1

            
            else:

                new_vel_list.append((lin_v, ang_v))
                pointer+=1
        
        lin_v, ang_v = my_list[-1]
        new_vel_list.append((lin_v, ang_v))

        return new_vel_list




    def action_set(self, node):
        next_node = []
        x_in = node[0][0]
        y_in = node[0][1]
        theta_in = node[1]
        # looping through action set
        count =0
        for action in [
            [self.rpm1, self.rpm1],[self.rpm2, self.rpm2],
                      #[self.rpm2, self.rpm1], [self.rpm1, self.rpm2],
                       [self.rpm1, 0], [0, self.rpm1],
                      #[self.rpm2, 0], [0, self.rpm2]
                       ]:
            UL = action[0]
            UR = action[1]
            x_val, y_val, theta_val, cost_val = self.cost(x_in, y_in, theta_in, UL, UR)

            if count ==0:
                cost_val=0.7*cost_val

            elif count ==1:
                cost_val=0.9*cost_val

            elif count ==2:
                cost_val=1*cost_val

            elif count ==3:
                cost_val=1*cost_val

            elif count ==4:
                cost_val=1.2*cost_val

            elif count ==5:
                cost_val=1.2*cost_val

            count +=1
            next_node_details = (x_val, y_val, theta_val, cost_val)
            # Check if the resulting position is within the map and not obstructed
            x, y = next_node_details[0], next_node_details[1]
            is_curve_free = False
            if -1 < x < self.width and 0 < y < self.height and self.map_data[y][x] == 0:
                is_curve_free = True
                for points in self.curve.copy():
                    xp, yp = points
                    if not (-1 < xp < self.width and 0 < yp < self.height and self.map_data[yp][xp] == 0):
                        is_curve_free = False
                        break
            if is_curve_free == True:
                self.lines[(x, y, x_in, y_in)] = self.curve.copy()
                self.velocities[(x, y, x_in, y_in)] = self.vel.copy()
                next_node.append(next_node_details)

        return next_node

    # Function to compute the curvature time
    def compute_curvature_time(self):
        theta=self.curvature_angle
        speed = 2*math.pi*self.rpm2/60
        if self.rpm1 >= self.rpm2:
            speed = 2*math.pi*self.rpm1/60
        theta = theta * 3.14/180
        self.fixed_time = (theta * self.wheel_distance) / \
            (speed*self.wheel_radius)

    # Function for cost calculation
    def cost(self, Xi, Yi, Thetai, UL, UR):
        t = 0
        r = self.wheel_radius
        L = self.wheel_distance
        dt = 0.1
        Xn = Xi
        Yn = Yi
        Thetan = 3.14 * Thetai / 180
        UL = 2*math.pi*UL/60
        UR = 2*math.pi*UR/60
        D = 0
        self.curve = []
        self.vel = []
        dt = self.fixed_time/100

        # looping for integration
        while t < self.fixed_time:
            t = t + dt
            # finding small change in x,y and theta
            Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Delta_Theta = (r / L) * (UR - UL) * dt
            Thetan += Delta_Theta
            Xn += Delta_Xn
            Yn += Delta_Yn
            D = D + math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),
                              2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt), 2))
            # appending the points for plotting
            self.curve.append((round(Xn), round(Yn)))
            Vel_Xn = Delta_Xn/(dt*100)
            Vel_Yn = Delta_Yn/(dt*100)
            Vel_theta = Delta_Theta/dt

            self.vel.append((round(Vel_Xn,5), round(Vel_Yn,5), round(Vel_theta,5)))   # m/s , m/s, rad
        Thetan = (180 * (Thetan) / 3.14)
        return round(Xn), round(Yn), round(Thetan), round(D, 3)

    # this function generates the animation

    def animate(self):
        # start_x, start_y = self.normal_coords_to_cv2(self.start_point[0], self.start_point[1])
        # goal_x, goal_y = self.normal_coords_to_cv2(self.goal_point[0], self.goal_point[1])
        start_x, start_y = self.start_point[0], self.start_point[1]
        goal_x, goal_y = self.goal_point[0], self.goal_point[1]
        node_incr = 0
        # draw goal point
        cv2.circle(self.map, (goal_x, goal_y), 6, (50, 50, 255), -1)
        # draw the threshold circle
        cv2.circle(self.map, (goal_x, goal_y), round(
            self.goal_threshold), (130, 130, 130), 1)
        # draw start point
        cv2.circle(self.map, (start_x, start_y), 6, (50, 255, 50), -1)

        # To display the node visit sequence
        for visited_node in self.visited_list:
            node_incr += 1
            xn, yn = visited_node[0], visited_node[1]
            # xn,yn= self.normal_coords_to_cv2(visited_node[0],visited_node[1])

            xv, yv = visited_node
            xp, yp = self.parent[visited_node]
            curve = self.lines[(xv, yv, xp, yp)]
            for point in curve:
                px, py = point
                # ppx, ppy = self.normal_coords_to_cv2(px, py)
                ppx, ppy = px, py

                cv2.circle(self.map, (ppx, ppy), 1, (150, 150, 150), -1)
            # To speed up the animation we update frame every 10 nodes
            # if node_incr % 25 == 0:
            cv2.imshow("Map", self.map)
            cv2.waitKey(1)

        # To display the path
        for node in self.optimal_path:

            # xn,yn= self.normal_coords_to_cv2(node[0],node[1])
            xn, yn = node[0], node[1]

            self.map[yn, xn] = (150, 150, 150)
            # xn,yn = self.normal_coords_to_cv2(node[0], node[1])
            xn, yn = node[0], node[1]
            xv, yv = node
            xp, yp = self.parent[node]
            if node != self.start_point:
                curve = self.lines[(xv, yv, xp, yp)]
                for point in curve:
                    px, py = point
                    # ppx, ppy = self.normal_coords_to_cv2(px, py)
                    ppx, ppy = px, py

                    cv2.circle(self.map, (ppx, ppy), 1, (0, 0, 0), -1)
            cv2.imshow("Map", self.map)
            cv2.waitKey(1)
            time.sleep(0.05)

        cv2.imshow("Map", self.map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Function to calculate the euclidean distance between 2 nodes.
    def euclidean_distance(self, node1, node2):
        d = math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)
        return round(d, 4)

    # Backtracking function
    def backtracking(self, final_node):
        # Initializing path list with goal points
        path = []
        # Current node is (node_index, parent_node_index, coordinates)
        current_node = final_node
        path.append(current_node)
        # Looping as long as we reach initial point
        while (current_node) != (self.start_point):
            parent_node = self.parent[current_node]
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
            # if (self.euclidean_distance(node_coordinates, self.goal_point) <= self.goal_threshold):
            #     print("Goal reached!")
            #     self.final_node = node_coordinates
            #     # call backtracking function
            #     self.optimal_path = self.backtracking(self.final_node)
            #     break

            if (self.goal_checker(node_coordinates)) :
                print("Goal reached!")
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
                added_edge_cost = adjacent_node[3]
                # Calculating cost to come
                updated_edge_cost = self.current_edge_cost[node_coordinates] + \
                    added_edge_cost
                # Calculating the cost to go
                heuristic_cost = self.euclidean_distance(
                    adjacent_node_coords, self.goal_point)*self.weight
                # Calculating the total cost
                total_cost = heuristic_cost + updated_edge_cost
                # add/update visited nodes
                if (adjacent_node_coords not in self.visited_list) or (total_cost < self.current_total_cost.get(adjacent_node_coords, float('inf'))):
                    # updating the cost
                    self.current_edge_cost[adjacent_node_coords] = updated_edge_cost
                    self.current_total_cost[adjacent_node_coords] = total_cost
                    # Calculating the orientation from 0 to 360.
                    orientation = ((360+adjacent_node[2]) % 360)
                    # Putting the nodes in queue
                    self.node_queue.put(
                        (total_cost, (adjacent_node_coords, orientation)))
                    # Updating parent dictionary for backtracking
                    self.parent[adjacent_node_coords] = node_coordinates
                    # adding node to visited list
                    self.visited_list.append(adjacent_node[:2])
                    self.node_orientation_dict[adjacent_node[:2]]=adjacent_node[2]

        data = self.return_path_data()
        return data


if __name__ == "__main__":
    planner = PathPlanning()
    data = planner.astar()
    planner.animate()

    time_size = data["update_rate"]
    vel_list = data["velocity"]
    print (time_size)
    print (len(vel_list)*time_size)



 
        
