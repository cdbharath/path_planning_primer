# Basic searching algorithms
# Solution by Bharath Kumar Ramesh Babu
# TODO corner cases start is obs, goal is obs, start == goal

# Class for each node in the grid
from locale import currency


class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

######################################################################
#   HELPER FUNCTIONS DEFINED BY ME

def init_node_matrix(grid, goal):
    '''
    intializes a matrix of node based on the grid values
    '''
    node_matrix = []
    for i in range(len(grid)):
        row = []
        for j in range(len(grid[0])):
            node = Node(i, j, bool(grid[i][j]), None)
            node.g = float("inf")
            node.cost = node.g
            node.h = abs(node.row - goal[0]) + abs(node.col - goal[1]) # manhattan distance
            row.append(node)
        node_matrix.append(row)
    return node_matrix

def is_safe(row, col, node_matrix, visited):
    '''
    check if the exploring node is safe
    '''
    if visited:
        return 0 <= row < len(node_matrix) and 0 <= col < len(node_matrix[0]) and not node_matrix[row][col].is_obs and not visited[row][col] 
    else:
        return 0 <= row < len(node_matrix) and 0 <= col < len(node_matrix[0]) and not node_matrix[row][col].is_obs

def backtrack_path(cur_node, path, node_matrix):
    '''
    returns the shortest path found by the algorithm
    '''
    while cur_node is not None: 
        path.append([cur_node.row, cur_node.col])
        cur_node = cur_node.parent
    return path[::-1]

#######################################################################

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    
    path = []
    steps = 0
    found = False

    # initialise matrix of nodes, visited matrix, start and goal node
    visited = [row[:] for row in grid]    # This can also be a list (reducing space complexiety)
    node_matrix = init_node_matrix(grid, goal)
       
    start_ = node_matrix[start[0]][start[1]]
    goal = node_matrix[goal[0]][goal[1]]
    
    visited[start[0]][start[1]] = 1
    queue = [start_]  # queue for bfs

    # loop until queue is empty    
    while len(queue) > 0:
        cur_node = queue.pop(0)
                
        # update step if the node is explored
        steps = steps + 1

        # terminate if the goal is reached         
        if cur_node.row == goal.row and cur_node.col == goal.col:
            found = True
            break 
                
        # discover neighbours, check if they are safe to visit and add them in the queue. set parents to the discovered node
        neighbours = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for neighbour in neighbours:
            if is_safe(cur_node.row + neighbour[0], cur_node.col + neighbour[1], node_matrix, visited):
                node = node_matrix[cur_node.row + neighbour[0]][cur_node.col + neighbour[1]]
                node.parent = cur_node

                # set the node as visited         
                visited[cur_node.row + neighbour[0]][cur_node.col + neighbour[1]] = 1
                queue.append(node)       
    
    # get the shortest path
    path = backtrack_path(goal, path, node_matrix)

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # initialise matrix of nodes, visited matrix, start and goal node
    visited = [row[:] for row in grid]    # This can also be a list (reducing space complexiety)
    node_matrix = init_node_matrix(grid, goal)
       
    start_ = node_matrix[start[0]][start[1]]
    goal = node_matrix[goal[0]][goal[1]]
    
    stack = [start_] # stack for dfs

    # loop until queue is empty    
    while len(stack) > 0:
        cur_node = stack.pop()
        
        # dont explore if visited already
        if visited[cur_node.row][cur_node.col] == 1:
            continue
        
        # update step if the node is explored
        steps = steps + 1

        # terminate if the goal is reached         
        if cur_node.row == goal.row and cur_node.col == goal.col:
            found = True
            break 
        
        # set the node as visited (outside the 'visiting neighbours' loop as per wikipedia)
        visited[cur_node.row][cur_node.col] = 1
        
        # discover neighbours, check if they are safe to visit and add them in the queue. set parents to the discovered node
        neighbours = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for neighbour in neighbours[::-1]:
            if is_safe(cur_node.row + neighbour[0], cur_node.col + neighbour[1], node_matrix, visited):
                node = node_matrix[cur_node.row + neighbour[0]][cur_node.col + neighbour[1]]
                node.parent = cur_node 
                stack.append(node)       
    
    # get the shortest path
    path = backtrack_path(goal, path, node_matrix)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
        
    # initialise matrix of nodes, visited matrix, start and goal node
    node_matrix = init_node_matrix(grid, goal)
       
    start_ = node_matrix[start[0]][start[1]]
    start_.g = 0  # initialize the 'cost to come' of start node to 0 
    start_.cost = start_.g
    goal = node_matrix[goal[0]][goal[1]]

    queue = [start_]  # queue for djikstra

    # loop until queue is empty    
    while len(queue) > 0:
        # pop the node with the least cost (alternative to priority queue, to not use library or pile up the code, but might be less efficient with time) 
        queue = sorted(queue, key=lambda x:x.cost)
        cur_node = queue.pop(0)
                
        # update step if the node is explored
        steps = steps + 1

        # terminate if the goal is reached         
        if cur_node.row == goal.row and cur_node.col == goal.col:
            found = True
            break 
                
        # discover neighbours, check if they are safe to visit and add them in the queue. set parents to the discovered node
        neighbours = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for neighbour in neighbours:
            if is_safe(cur_node.row + neighbour[0], cur_node.col + neighbour[1], node_matrix, False):
                node = node_matrix[cur_node.row + neighbour[0]][cur_node.col + neighbour[1]]
                if node.g > cur_node.g + 1:   # check if the cost of the neighbour is greater than current node to update the cost 
                    node.g = cur_node.g + 1
                    node.cost = node.g              # total cost is just the 'cost to come'
                    node.parent = cur_node 
                    queue.append(node)       
    
    # get the shortest path
    path = backtrack_path(goal, path, node_matrix)

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # initialise matrix of nodes, visited matrix, start and goal node
    node_matrix = init_node_matrix(grid, goal)
       
    start_ = node_matrix[start[0]][start[1]]
    start_.g = 0  # initialize the 'cost to come' of start node to 0
    goal = node_matrix[goal[0]][goal[1]]
    start_.cost = start_.g + start_.h

    queue = [start_]  # queue for bfs

    # loop until queue is empty    
    while len(queue) > 0:
        # pop the node with the least cost (alternative to priority queue, to not use library or pile up the code, but might be less efficient with time) 
        queue = sorted(queue, key=lambda x:x.cost)
        cur_node = queue.pop(0)
                
        # update step if the node is explored
        steps = steps + 1

        # terminate if the goal is reached         
        if cur_node.row == goal.row and cur_node.col == goal.col:
            found = True
            break 
                
        # discover neighbours, check if they are safe to visit and add them in the queue. set parents to the discovered node
        neighbours = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for neighbour in neighbours:
            if is_safe(cur_node.row + neighbour[0], cur_node.col + neighbour[1], node_matrix, False):
                node = node_matrix[cur_node.row + neighbour[0]][cur_node.col + neighbour[1]]
                if node.g > cur_node.g + 1:   # check if the cost of the neighbour is greater than current node to update the cost
                    node.g = cur_node.g + 1
                    node.cost = node.g + node.h     # total cost = 'cost to come' + 'cost to reach goal'
                    node.parent = cur_node 
                    queue.append(node)       
    
    # get the shortest path
    path = backtrack_path(goal, path, node_matrix)

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
