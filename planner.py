# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
TRAJECTORY="parabola"

import numpy as np

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # parabola - y = x^2 (0 <= x <= 1.5) 
        if TRAJECTORY == "parabola":
            parabola_traj = [[x, x**2] for x in np.linspace(0, 1.5, 50)]
            return parabola_traj
        
        if TRAJECTORY == "sigmoid":
            # sigmoid - y = 2/(1+exp(-2x)) -1 (0 <=x <= 2.5)
            sigmoid_traj = [[x, 2/(1+np.exp(-2*x)) - 1] for x in np.linspace(0, 2.5, 50)]
            return sigmoid_traj

        raise ValueError("Unknown trajectory type: {}".format(TRAJECTORY))


