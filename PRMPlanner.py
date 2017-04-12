from __future__ import division
import math
import numpy as np
import random
from graph import Graph, Edge
from search_classes import SearchNode, Path
from utils import *
from shapely.geometry import Point, LineString
    
class PRMPathPlanner():
    def initialise(self, environment, bounds, start_pose, goal_region, radius, resolution, isLazy):
        self.env = environment
        self.bounds = bounds
        self.radius = radius
        self.resolution = resolution
        self.isLazy = isLazy
        self.goal_region = goal_region
        self.start_pose = start_pose
        
    def path (self,environment, bounds, q_init_point, q_goal_region, radius, resolution, isLazy):
        
        self.initialise(environment, bounds, q_init_point, q_goal_region, radius, resolution, isLazy)
        
        x0, y0 = q_init_point
        x1, y1 = q_goal_region.centroid.coords[0]
        q_init = (x0, y0)
        q_goal = (x1, y1)
        V = set()
        E = set()
        
        if q_init == q_goal:
            path = [q_init, q_goal]
            V.union([q_init, q_goall])
            E.union([(q_init, q_goal)])
        elif self.isEdgeCollisionFree(q_init, q_goal, self.radius):
            path = [q_init, q_goal]
            V.union([q_init, q_goal])
            E.union([(q_init, q_goal)])
        else:
            path, V, E = self.PRMSearch(self.bounds, q_init, self.radius, q_goal, isLazy)
            
        '''
        if path != None:
            print "path found"
        else:
            print path
        '''
        return path, V, E
    
    def PRMSearch(self, bounds, q_init, radius, q_goal, isLazy):
        '''print "starting roadmap construction"'''
        # number of nodes to put in the roadmap
        n = min(max(len(self.env.obstacles) *2, 500), 2000)
        # number of closest neighbours to examine for each configuration
        k = min(int(n/50),15)
        
        # construct the probabilistic roadmap
        V,E = self.roadmapConstruction(bounds, radius, k, n, isLazy)
        
        # k closest neighbours of start configuration q_init, from V, sorted by distance
        Nqinit = self.find_k_closest_neighbours(V, q_init, k)
        # k closest neighbours of goal configuration q_goal, from V, sorted by distance
        Nqgoal = self.find_k_closest_neighbours(V, q_goal, k)
        # V is the union of itself and Nqinit and Nqgoal
        V = V.union(Nqinit, Nqgoal)
        
        # iterate through Nqinit and try to find the closest neighbour to q_init that has a collision free edge between them
        for qprime in Nqinit:
            if self.isEdgeCollisionFree(q_init, qprime, radius):
                E.add((q_init, qprime))
                break

        # iterate through Nqgoal and try to find the closest neighbour to q_goal that has a collision free edge between them
        for qprime in Nqgoal:
            if self.isEdgeCollisionFree(q_goal, qprime, radius):
                E.add((q_goal, qprime))
                break
        '''print "starting search"'''
        # find, using A* Search, the shortest path between our intial point and goal point, given the constructed graph.
        path = None
        if isLazy:
            ok_edges = set()
            attempt = 1
            
            while True:
                
                path_found = True
                path = self.findShortestPath(V, E, q_init, q_goal)
                
                if path == None:
                    if attempt == 5:
                        break
                    '''print attempt'''
                    n += 200
                    V, E = self.roadmapReconstruction(bounds, radius, k, n, isLazy, V, E, q_init, q_goal)
                    attempt += 1
                    continue
                
                for i in xrange(len(path)-1):
                    edge = (path[i], path[i+1])
                    reversed_edge = (path[i+1], path[i])
                    
                    if edge not in ok_edges:
                        if not self.isEdgeCollisionFree(path[i], path[i+1], radius):
                            if edge in E:
                                E.remove(edge)
                            if reversed_edge in E:
                                E.remove(reversed_edge)
                            path_found = False
                        else:
                            ok_edges.add(edge)
                            ok_edges.add(reversed_edge)
                if path_found:
                    break
                
                    
        else:
            for attempt in xrange(1,6):
                '''print attempt'''
                path = self.findShortestPath(V, E, q_init, q_goal)
                if path == None:
                    if attempt == 5:
                        break
                    n += 200
                    V, E = self.roadmapReconstruction(bounds, radius, k, n, isLazy, V, E, q_init, q_goal)
                else:
                    break
        
        # returns path: a list of tuples representing points along the shortest path.
        return path, V, E
            
        
        
    def roadmapConstruction(self, bounds, radius, k, n, isLazy):
        
        # Vertices in our roadmap
        V = set()
        # Edges in our roadmap
        E = set()
        '''print "start finding n free points"'''
        while len(V) < n:
            # random collision free configuration in our configuration space
            q = self.find_random_collision_free_configuration(bounds, radius)
            # V is the union of itself and the n nodes we put into our roadmap
            V.add(q)
        '''print "starting finding k nearest neighbours"'''
        for q in V:
            # k closest neighbours of configuration q, from V, sorted by distance
            Nq = self.find_k_closest_neighbours(V, q, k)
            # set of collision free edges between q and its k closest neighbours
            q_edges = self.find_collision_free_edges(q, Nq, radius, E, isLazy)
            # E is the union of itself and the newly found edges
            E = E.union(q_edges)
        
        # retuns V,E: the set of n Vertices and corresponding connected edges that form the probabilistic roadmap
        return V,E
           
        
    def roadmapReconstruction(self, bounds, radius, k, n, isLazy, V, E, q_init, q_goal):
        '''print "start finding n free points"'''
        V_new_points = set()
        while len(V) < n:
            # random collision free configuration in our configuration space
            q = self.find_random_collision_free_configuration(bounds, radius)
            # V is the union of itself and the n nodes we put into our roadmap
            V.add(q)
            V_new_points.add(q)
        '''print "starting finding k nearest neighbours"'''
        for q in V_new_points:
            # k closest neighbours of configuration q, from V, sorted by distance
            Nq = self.find_k_closest_neighbours(V, q, k)
            # set of collision free edges between q and its k closest neighbours
            q_edges = self.find_collision_free_edges(q, Nq, radius, E, isLazy)
            # E is the union of itself and the newly found edges
            E = E.union(q_edges)
            
        # k closest neighbours of start configuration q_init, from V, sorted by distance
        Nqinit = self.find_k_closest_neighbours(V, q_init, k)
        # k closest neighbours of goal configuration q_goal, from V, sorted by distance
        Nqgoal = self.find_k_closest_neighbours(V, q_goal, k)
        # V is the union of itself and Nqinit and Nqgoal
        V = V.union(Nqinit, Nqgoal)
        
        for qprime in Nqinit:
            #if (q_init, qprime) not in E and (qprime, q_init) not in E:
                if self.isEdgeCollisionFree(q_init, qprime, radius):
                    E.add((q_init, qprime))
                    break

        for qprime in Nqgoal:
            #if (q_goal, qprime) not in E and (qprime, q_goal) not in E:
                if self.isEdgeCollisionFree(q_goal, qprime, radius):
                    E.add((q_goal, qprime))
                    break
                
        return V,E
        
        
    def find_random_collision_free_configuration(self, bounds, radius):
        # continue until a point is found
        while True:
            # get a random point
            q = self.get_random_point(bounds)
            # verify if it is collision free with all obstacles and whether it is within the bounds of the problem space
            if self.isPointCollisionFree(q, radius) and not self.isOutOfBounds(q, bounds, radius) :
                return q
    
    
    def find_k_closest_neighbours(self, V, q, k):
        # convert V to a list so that we can index elements inside
        list_V = list(V)
        
        # initialise list to store euclidian distance between every point v in V and q
        neighbour_distances = []
        
        # calculate the distances and populate the list
        for v in list_V:
            neighbour_distances.append(self.euclidianDist(q, v))
            
        # sort the neighbours_distances list and extract their position index in V
        sorting_indexes = np.argsort(neighbour_distances)
        
        # extract k closest neighbours to q, by extracting the elements from V, with indexes in the 1:k+1 slots of sorting_indexes.
        # Note: We ignore the first index in sorting_indexes, because this will simply index the point q in V
        k_closest_neighbours = [list_V[sorting_indexes[i]] for i in range(1,k+1)]
        
        # we return a set containing the k closest neighbours, so that the union can be taken with V
        return set(k_closest_neighbours)

    
    # finds the set of all edges between q and its neighbours in Nq, that are collision free
    def find_collision_free_edges(self, q, Nq, radius, E, isLazy):
        # initialise set that will store all collision free edges
        edges = set()
        # iterate through all neighbours of q in Nq and find collision free edges
        for neighbour in Nq:
            if (q, neighbour) not in E and (neighbour,q) not in E:
                # add the tuple edge to edges if it is collision free. edge  = (q, neighbour)
                if isLazy:
                    edges.add((q, neighbour))
                else:
                    if self.isEdgeCollisionFree(q, neighbour, radius):
                        edges.add((q, neighbour))
        # return the set of edges so that it's union may be taken with E
        return edges
        
    
    # checks if the edge between two points in our configuration space have a collision free edge
    def isEdgeCollisionFree(self, q_init, qprime, radius):
        # generate a line between the two points using the shapely library
        line = LineString([q_init, qprime])
        # buffer the line so that it represents the physicality (shape) of the object
        expanded_line = line.buffer(radius)
        # iterate through all obstacles in the environment and verify if the buffered edge interesects any one of them
        for obstacle in self.env.obstacles:
            # terminate false as soon as intersection exists
            if expanded_line.intersects(obstacle):
                return False
        # terminate true if no intersection exists
        return True

    

    def findShortestPath(self, V, E, start_state, goal_state):
        graph = self.generateGraph(V,E)
        problem = GraphSearchProblem(graph, start_state, goal_state)
        return self.astar_search(problem, self.heuristic_to_goal)
    
    
    def generateGraph(self, V, E):
        graph = Graph()
        for edge in E:
            graph.add_edge(edge[0], edge[1], self.euclidianDist(edge[0], edge[1]))
        vertex_locations_dictionary = dict()
        for vertex in V:
            vertex_locations_dictionary[vertex] = vertex
        graph.set_node_positions(vertex_locations_dictionary)
        return graph

    
    def astar_search(self, problem, h):
        cost_func = lambda x: x.cost
        f = lambda x: cost_func(x) + h(x, problem.goal)
        return self.best_first_search(problem, f)
    
    def best_first_search(self, problem, f):
        queue = PriorityQueue(f=f)
        queue.append(SearchNode(problem.start))
        expanded = set([problem.start])
        max_queue = 1
        path = list()
        while queue:
            current_node = queue.pop()
            expanded.add(current_node.state)
            
            if(current_node.state == problem.goal):
                final_path = Path(current_node).path
                return final_path
            
            expanded_sn = problem.expand_node(current_node)
            
            for sn in expanded_sn:
                if(sn.state not in expanded):
                    if sn in queue:
                        if(sn.cost < queue[sn].cost):
                            del queue[sn]
                            queue.append(sn)
                    else:
                        queue.append(sn)
            if(len(queue) > max_queue):
                max_queue = len(queue)
        # If we get to here, no solution has been found.
        return None
    
    
    def heuristic_to_goal(self, search_node, goal_state):
        return self.euclidianDist(search_node.state, goal_state)
        
    # find a random point within the configuration space
    # Note: it finds an actual point and not a buffered point that represents the phsicality of our object
    def get_random_point(self, bounds):
        x = bounds[0] + random.random()*(bounds[2]-bounds[0])
        y = bounds[1] + random.random()*(bounds[3]-bounds[1])
        return (x, y)
        
    # check if configuration point q is collision free with all obstacles in the environment
    def isPointCollisionFree(self, q, radius):
        # buffer the point using the shapely class to check if the physical representation of q intersects any obstacles
        buffered_point = Point(q).buffer(radius)
        # verify collisions with every obstacle in the environment
        for obstacle in self.env.obstacles:
            # if q is in any obstacle, terminate false
            if obstacle.intersects(buffered_point):
                return False
        # if no obstacle contains q, then terminate true
        return True
    
    
    # check if configuration point q is out of bounds
    def isOutOfBounds(self, q, bounds, radius):
        # left boundary check
        if((q[0]-radius) < bounds[0]):
            return True
        # bottom boundary check
        if((q[1]-radius) < bounds[1]):
            return True
        # right boundary check
        if((q[0]+radius) > bounds[2]):
            return True
        # top boundary check
        if((q[1]+radius) > bounds[3]):
            return True
        # if all checks fail, then ball q wih given radius is within the boundary
        return False
    
    # calculate the euclidian distance between two configuration points q1, q2
    def euclidianDist(self, q1, q2):
        return math.sqrt((q2[0]-q1[0])**2 + (q2[1]-q1[1])**2)
    
    
class GraphSearchProblem(object):
    def __init__(self, graph, start, goal):
        self.graph = graph
        self.start = start
        self.goal = goal
    def test_goal(self, state):
        return self.goal == state
    def expand_node(self, search_node):
        """Return a list of SearchNodes, having the correct state, parent and updated cost."""
        current_node = search_node.state
        current_cost = search_node.cost
        outgoing_edges = self.graph.node_edges(current_node)
        expanded_sn = [SearchNode(edge.target, search_node, current_cost + edge.weight) for edge in outgoing_edges ]
        return expanded_sn
    