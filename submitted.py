# submitted.py
# ---------------
# Licensing Information:
# This HW is inspired by previous work by University of Illinois at Urbana-Champaign


"""
This is the main entry point for MP5. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# submitted should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi)

#SOURCES: https://docs.python.org/3/ , https://www.redblobgames.com/pathfinding/a-star/implementation.html, https://en.wikipedia.org/wiki/Breadth-first_search,
# https://en.wikipedia.org/wiki/A*_search_algorithm, https://en.wikipedia.org/wiki/Depth-first_search, https://chatgpt.com/ 


from collections import deque
import heapq

def bfs(maze):
    start = maze.start
    goal = maze.waypoints[0]
    queue = deque([(start, [start])])
    visited = set()
    visited.add(start)

    while queue:
        (current, path) = queue.popleft()
        if current == goal:
            return path
        
        for neighbor in maze.neighbors(*current):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))

    return []

def dfs(maze):
    start = maze.start
    goal = maze.waypoints[0]
    stack = [(start, [start])]
    visited = set()
    visited.add(start)

    while stack:
        (current, path) = stack.pop()
        if current == goal:
            return path
        
        for neighbor in maze.neighbors(*current):
            if neighbor not in visited:
                visited.add(neighbor)
                stack.append((neighbor, path + [neighbor]))

    return []

def heuristic(a, b):
    # Using Manhattan distance as the heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_single(maze):
    start = maze.start
    goal = maze.waypoints[0]
    pq = []
    heapq.heappush(pq, (0 + heuristic(start, goal), 0, start, [start]))
    visited = set()

    while pq:
        (f, cost, current, path) = heapq.heappop(pq)
        if current == goal:
            return path
        
        if current in visited:
            continue
        visited.add(current)

        for neighbor in maze.neighbors(*current):
            if neighbor not in visited:
                new_cost = cost + 1
                heapq.heappush(pq, (new_cost + heuristic(neighbor, goal), new_cost, neighbor, path + [neighbor]))

    return []
