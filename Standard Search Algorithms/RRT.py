# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx
from scipy import spatial


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.vertices_coord = []
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices_coord = []
        self.vertices.append(self.start)
        self.vertices_coord.append(tuple([self.start.row, self.start.col]))

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        point1 = np.array([node1.row, node1.col])
        point2 = np.array([node2.row, node2.col])
        return np.linalg.norm(np.array(point1) - np.array(point2))

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        p1 = np.array([node1.row, node1.col])
        p2 = np.array([node2.row, node2.col])
        
        p = p1
        d = p2-p1
        N = np.max(np.abs(d))

        if N == 0:
            return True

        s = d/N
        
        for ii in range(0,N):
            p = p+s

            if not self.map_array[int(p[0])][int(p[1])]:
                return True
        
        return False


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        x = int(np.random.rand()*self.size_row)
        y = int(np.random.rand()*self.size_col)

        return [[x, y], [self.goal.row, self.goal.col]][np.random.choice(2, p=[1 - goal_bias, goal_bias])]

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        point = np.array(point)

        tree = spatial.KDTree(self.vertices_coord)
        nearest_dist, nearest_ind = tree.query(point, k=1)

        ### YOUR CODE HERE ###
        return self.vertices[nearest_ind]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        new_node = np.array(new_node)

        kdtree = spatial.KDTree(self.vertices_coord)
        connects = kdtree.query_ball_point(new_node, neighbor_size)

        vertices = np.array(self.vertices)

        return vertices[connects]


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###

        # Brute force rewire
        min_cost = float("inf")
        min_idx = -1
        for i in range(len(neighbors)):
            if neighbors[i].cost + self.dis(new_node, neighbors[i]) < min_cost and not self.check_collision(new_node, neighbors[i]):
                min_cost = neighbors[i].cost + self.dis(new_node, neighbors[i])
                min_idx = i
        
        if len(neighbors) and min_idx != -1:
            new_node.parent = neighbors[min_idx]
            new_node.cost = min_cost

        for i in range(len(neighbors)):
            if new_node.cost + self.dis(new_node, neighbors[i]) < neighbors[i].cost and not self.check_collision(new_node, neighbors[i]):
                neighbors[i].parent = new_node
                neighbors[i].cost = new_node.cost + self.dis(new_node, neighbors[i])
        
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        step_size = 20 # For extending
        for i in range(n_pts):
            # Gets new point
            row, col = self.get_new_point(0.5)

            # Extend towards the point 
            nearest_node = self.get_nearest_node([row, col])

            # Point to extend
            angle = math.atan2((nearest_node.row - row), nearest_node.col - col)
            new_node = Node(int(nearest_node.row - step_size*math.sin(angle)), int(nearest_node.col - step_size*math.cos(angle)))
            
            if not 0 <= new_node.row < self.size_row or not 0 <= new_node.col < self.size_col or not self.map_array[new_node.row][new_node.col]:
                continue 

            if self.check_collision(new_node, nearest_node):
                continue

            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.dis(new_node, nearest_node)
            self.vertices.append(new_node)
            self.vertices_coord.append(tuple([new_node.row, new_node.col]))

            if self.dis(new_node, self.goal) < 30 and not self.check_collision(new_node, self.goal):
                self.vertices.append(self.goal)
                self.vertices_coord.append(tuple([self.goal.row, self.goal.col]))
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.dis(new_node, self.goal)
                self.found = True
                break

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()
        

    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        step_size = 15 # For extending
        for i in range(n_pts):
            # Gets new point
            row, col = self.get_new_point(0.5)

            # Extend towards the point 
            nearest_node = self.get_nearest_node([row, col])

            # Point to extend
            # if nearest_node.col == col:
            #     continue

            # slope = (nearest_node.row - row)/(nearest_node.col - col)
            # angle = math.atan(slope)

            # # Create new sample
            # if row >= nearest_node.row:
            #     new_node = Node(int(nearest_node.row + 20*math.cos(angle)), int(nearest_node.col + 20*math.sin(angle)))
            # else:
            #     new_node = Node(int(nearest_node.row - 20*math.cos(angle)), int(nearest_node.col - 20*math.sin(angle)))

            angle = math.atan2((nearest_node.row - row), nearest_node.col - col)
            new_node = Node(int(nearest_node.row - step_size*math.sin(angle)), int(nearest_node.col - step_size*math.cos(angle)))

            if not 0 <= new_node.row < self.size_row or not 0 <= new_node.col < self.size_col or not self.map_array[new_node.row][new_node.col]:
                continue 

            if self.check_collision(new_node, nearest_node):
                continue

            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.dis(new_node, nearest_node)

            # Smoothen the path
            neighbors = self.get_neighbors([new_node.row, new_node.col], neighbor_size)
            self.rewire(new_node, neighbors)

            self.vertices.append(new_node)
            self.vertices_coord.append(tuple([new_node.row, new_node.col]))

            if self.dis(new_node, self.goal) < 20 and not self.check_collision(new_node, self.goal):
                self.vertices.append(self.goal)
                self.vertices_coord.append(tuple([self.goal.row, self.goal.col]))
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.dis(new_node, self.goal)
                self.found = True
                # break

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        print("Done")
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
