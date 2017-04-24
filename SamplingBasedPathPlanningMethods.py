from RRTFamilyOfPlanners import RRTFamilyPathPlanner
from PRMPlanner import PRMPathPlanner
from drawer import draw_results
import time

class SamplingBasedPathPlanner():

    """Plans path using a sampling based algorithm on a 2D environment.

    Contains methods for simple RRT based search, RRTstar based search, informed RRTstar based search, and PRM based search, all in 2D space.
    Methods also have the option to draw the results.

    """
    def __init__(self):
        """
        The planner contains two objects. One for planning using RRT algorithms and another for using a PRM planner.
        """
        self.RRTFamilySolver = RRTFamilyPathPlanner()
        self.PRMSolver = PRMPathPlanner()

    def RRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using RRT.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): Optional, if True return the first path found without having to sample all num_iterations points.
            drawResults (bool): Optional, if set to True it plots the path and enviornment using a matplotlib plot.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E

    def RRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using RRT*.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): Optional, if True return the first path found without having to sample all num_iterations points.
            drawResults (bool): Optional, if set to True it plots the path and enviornment using a matplotlib plot.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT*")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT*", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E

    def InformedRRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using informed RRT*.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            drawResults (bool): Optional, if set to True it plots the path and enviornment using a matplotlib plot.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations=True, RRT_Flavour="InformedRRT*")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("InformedRRT*", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E

    def PRM(self, environment, bounds, start_pose, goal_region, object_radius, resolution=3, isLAzy=True, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using PRM.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            isLazy (bool): Optional, if True it adds edges to the graph regardless of whether it collides with an obstacle and only checks for collisions when building the path.
            drawResults (bool): Optional, if set to True it plots the path and enviornment using a matplotlib plot.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the graph.
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the graph.
        """
        start_time = time.time()
        path, V, E = self.PRMSolver.path(environment, bounds, start_pose, goal_region, object_radius, resolution, isLAzy)
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("PRM", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E





