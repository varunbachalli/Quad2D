from quadcopter import QuadCopter2D
from simulator import Simulation
from casadiTrajectoryPlanner import TrajectoryOptimization
from MPC_SetPoint import MPC_SetPoint
from MPC_TrajectoryFollower import MPC_TrajectoryFollower
import numpy as np

if __name__ == "__main__":
    q = QuadCopter2D(l = 1.5, start_orientation=0, start_position=np.ones(2))
    delT = 0.1
    c = MPC_TrajectoryFollower(q, 20, delT)
    t = TrajectoryOptimization(q, 150)
    target = np.zeros_like(q.states)
    target[3:5] = [5,4]
    s = Simulation(quad = q, controller = c,trajectoryPlanner = t,target = target, delT = delT)
    s.start_animation()