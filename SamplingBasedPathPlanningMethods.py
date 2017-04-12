from RRTFamilyOfPlanners import RRTFamilyPathPlanner
from PRMPlanner import PRMPathPlanner
from drawer import draw_results
import time

class SamplingBasedPathPlanner():
    def __init__(self):
        self.RRTFamilySolver = RRTFamilyPathPlanner()
        self.PRMSolver = PRMPathPlanner()
        
    def RRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E
    
    def RRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT*")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT*", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E
    
    def InformedRRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, drawResults=False):
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations=True, RRT_Flavour="InformedRRT*")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("InformedRRT*", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E
    
    def PRM(self, environment, bounds, start_pose, goal_region, object_radius, resolution=3, isLAzy=True, drawResults=False):
        start_time = time.time()
        path, V, E = self.PRMSolver.path(environment, bounds, start_pose, goal_region, object_radius, resolution, isLAzy)
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("PRM", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E
    
    


    