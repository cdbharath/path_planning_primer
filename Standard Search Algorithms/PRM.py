# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        p = p1
        d = p2-p1
        N = np.max(np.abs(d))
        s = d/N
        
        for ii in range(0,N):
            p = p+s

            if not self.map_array[int(p[0])][int(p[1])]:
                return True
        
        return False


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        point1 = np.array(point1)
        point2 = np.array(point2)
        return np.linalg.norm(np.array(point1) - np.array(point2))


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        x = np.arange(0, self.size_row, self.size_row/np.sqrt(n_pts)).astype("int")
        y = np.arange(0, self.size_col, self.size_row/np.sqrt(n_pts)).astype("int")

        mesh = np.array(np.meshgrid(x, y))
        all_samples = mesh.T.reshape(-1, 2)       
        
        for sample in all_samples:
            if self.map_array[sample[0]][sample[1]]:
                self.samples.append(tuple(sample))
    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        x = np.random.randint(0, self.size_row, size=n_pts)
        y = np.random.randint(0, self.size_col, size=n_pts)
        
        all_samples = np.hstack((np.atleast_2d(x).T, np.atleast_2d(y).T)).tolist()

        for sample in all_samples:
            if self.map_array[sample[0]][sample[1]]:
                self.samples.append(tuple(sample))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        x = (np.random.rand(n_pts)*self.size_row).astype("int")
        y = (np.random.rand(n_pts)*self.size_col).astype("int")
        
        all_samples = np.hstack((np.atleast_2d(x).T, np.atleast_2d(y).T)).tolist()

        for sample in all_samples:
            if not self.map_array[sample[0]][sample[1]]:
                x = int(np.random.normal(sample[0], 25))
                y = int(np.random.normal(sample[1], 25))
                if 0 <= x < self.size_row and 0 <= y < self.size_col and self.map_array[x][y]:
                    self.samples.append((x, y))

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        x = (np.random.rand(n_pts)*self.size_row).astype("int")
        y = (np.random.rand(n_pts)*self.size_col).astype("int")
        
        all_samples = np.hstack((np.atleast_2d(x).T, np.atleast_2d(y).T)).tolist()

        for sample in all_samples:
            if not self.map_array[sample[0]][sample[1]]:
                x = int(np.random.normal(sample[0], self.size_row/10))
                y = int(np.random.normal(sample[1], self.size_col/10))
                if 0 <= x < self.size_row and 0 <= y < self.size_col and not self.map_array[x][y]:
                    if self.map_array[int((x + sample[0])/2)][int((y + sample[1])/2)]:
                        self.samples.append((int((x + sample[0])/2), int((y + sample[1])/2)))

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        kdtree = spatial.KDTree(self.samples)
        if sampling_method == "gaussian":
            pairs_wo_weights = kdtree.query_pairs(30)
        else:
            pairs_wo_weights = kdtree.query_pairs(20)
        pairs = []
        for pair in pairs_wo_weights:
            if not self.check_collision(self.samples[pair[0]], self.samples[pair[1]]):
                pairs.append((pair[0], pair[1], int(self.dis(self.samples[pair[0]], self.samples[pair[1]]))))            

        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)

        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])
        ### YOUR CODE HERE ###
        kdtree = spatial.KDTree(self.samples)
        connects_to_start = kdtree.query_ball_point(start, 100)
        connects_to_goal = kdtree.query_ball_point(goal, 100)
        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        start_pairs = []
        goal_pairs = []
        for connect in connects_to_start:
            if not self.check_collision(start, self.samples[connect]):
                start_pairs.append(('start', connect, int(self.dis(self.samples[-2], self.samples[connect]))))
        for connect in connects_to_goal:
            if not self.check_collision(goal, self.samples[connect]):
                goal_pairs.append(('goal', connect, int(self.dis(self.samples[-1], self.samples[connect]))))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        