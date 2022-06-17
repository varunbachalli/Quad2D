import matplotlib.pyplot as plt
import numpy as np
from quadcopter import QuadCopter2D
from matplotlib.animation import FuncAnimation
from MPC_SetPoint import MPC_SetPoint
from MPC_TrajectoryFollower import MPC_TrajectoryFollower
from casadiTrajectoryPlanner import TrajectoryOptimization

'''
assumed in the simulation that z = 0 is ground.
'''

class Simulation:
    '''
    takes a quad copter to simulate.
    '''
    def __init__(self, quad :QuadCopter2D, 
                       controller : MPC_TrajectoryFollower,
                       trajectoryPlanner : TrajectoryOptimization,
                       target : np.array,
                       delT = 0.01):
        self.delT = delT
        self.quad = quad
        self.controller = controller
        self.targetposition = target
        self.trajectoryPlanner = trajectoryPlanner
        self.trajectoryPlanner.SetTrajectory(self.quad.states, self.targetposition)
        self.time = 0.0

    '''
    function that updates the position of the quad copter in question with the appropriate torques
    '''
    def update(self):
        # F = self.controller.GetControlInput(startState = self.quad.states + np.random.uniform(-0.2,0.2,6), 
        #                                     targetState = self.targetposition)
        # F = self.controller.GetControlInput(startState = self.quad.states + np.random.uniform(-0.2,0.2,6), 
        #                                     targetState = self.targetposition)
    
        F = self.controller.GetControlInput(startState = self.quad.states + np.random.uniform(-0.3,0.3,6), 
                                            trajectoryOptimizer = self.trajectoryPlanner,
                                            time = self.time)
        
        self.quad.update_position(applyForce = F, delT = self.delT)
        self.time+= self.delT

    '''
    
    '''
    def get_plots(self):
        return [self.quad.getPlot(), self.trajectoryPlanner.getPlots()]
    
    # loop and check if any of the failure modes got met.
    def break_condition_met(self):
        return False


    '''
    function that shows the animation itself.
    '''
    def animation(self, frame):
        self.update()
        self.xdata, self.ydata = self.get_plots()[0]
        self.ln.set_data(self.xdata, self.ydata)
        
        if(self.break_condition_met()):
            print("stopping!")
            self.ani.event_source.stop()
            plt.close()
        return self.ln,



    def init(self):
        self.ax.set_xlim(-2, 7)
        self.ax.set_ylim(-2, 7)
        self.ax.set_aspect(1)
        self.time = 0
        return self.ln,

        
    '''
    function that begins the animation.
    '''
    def start_animation(self):
        self.fig, self.ax = plt.subplots()
        self.xdata, self.ydata = [], [] # have seperate data for seperate plot object
        self.ln, = plt.plot([], []) # update this to be as many as the number of objects
        self.ani = FuncAnimation(self.fig, self.animation, frames=np.linspace(0, 2*np.pi, 128),
                                 init_func=self.init, blit=False, interval = 30)
        plt.show()
        

