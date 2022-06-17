import numpy as np
from quadcopter import QuadCopter2D
from scipy.optimize import minimize, NonlinearConstraint, Bounds
import matplotlib.pyplot as plt


class TrajectoryOptimization:
    '''
    used to get the quad parameters like I, l ,m
    '''
    def __init__(self, quad : QuadCopter2D):
        self.l = quad.l
        self.mass = quad.mass
        self.inertia = quad.inertia
        self.currentState = quad.states 
        self.numStates = len(quad.states)
        self.numControls = quad.numControls 
        self.state_trajectory = []
        self.control_trajectroy = []
        # self.timeToTrajectory = 10 # seconds . making it an optimization parameter.
        self.steps = 30 # num steps in 3 seconds. number of optimization steps
        self.totalStates = self.steps * self.numStates
        self.totalOptimizationVariables = self.steps * (self.numControls + self.numStates) +  1 # total time
        self.totalControls = self.numControls * self.steps
        self.g = 9.81
        self.posBounds = (-np.inf, np.inf)
        self.angleBounds = (-np.pi/3, np.pi/3)
        self.velBounds = (-2,2)
        self.timeBounds = (0, np.inf)
        self.rotVelBounds = (-np.pi/12, np.pi/12)
        self.controlBounds = ((-10*self.g,10*self.g), (-10*self.g,10*self.g))
        self.stateBounds = (self.velBounds, self.velBounds, self.rotVelBounds, self.posBounds, self.posBounds, self.angleBounds)
        self.bounds = self.stateBounds*self.steps + self.controlBounds*self.steps + tuple([self.timeBounds])
        self.collocationConstraints = tuple(self.CollocationStateContstraints())
    '''
    using trapezoidal quadtrature of control imputs to create the trajectory of the quad

    objective values : [state0, state1,....., stateT, control0, control1, ... controlN]
    '''
    def objective(self,x):
        
        finalTime = -1 # last variable

        # minimize time taken to reach the goal.
        # use the trapezoid quadrature ∑ hk/2 * (w_k + w_(k+1)) where w_k = uk^2
        # = hk/2 * (∑ w_k + ∑ w_(k+1))   

        uk_start = self.totalStates
        uk_end = self.totalStates + self.totalControls - self.numControls
        uk1_start = self.totalStates + self.numControls
        uk1_end = self.totalStates + self.totalControls

        # add constraint that the next step shouldn't be too far from the current step
        # ∑ (x[k + 1] - x[k]) ^2
        # 
        xk_start = 0
        xk_1_start = self.numStates
        xk_end = self.totalStates - self.numStates
        xk_1_end = self.totalStates
        # state_change_constraint = x[]
        return x[finalTime]/2*self.steps *(np.dot(x[uk_start : uk_end], x[uk_start : uk_end]) + np.dot(x[uk1_start : uk1_end], x[uk1_start : uk1_end]))\
              +x[finalTime] * x[finalTime]  + np.dot(x[xk_1_start : xk_1_end] - x[xk_start: xk_end],x[xk_1_start : xk_1_end] - x[xk_start: xk_end])
    
    '''
    given state and control it'll evaluate the function 
    '''
    def model(self, x, u):
        # x_dot = f(x)
        return np.array([(u[0] + u[1]) * np.sin(x[5])/self.mass, # dx
                         (u[0] + u[1]) * np.cos(x[5])/self.mass - self.g,  #dy
                         (u[0] - u[1])/ (self.l * self.inertia),#dtheta
                         x[0], # x
                         x[1], # y 
                         x[2]]) # theta

    
    def CollocationStateContstraints(self):
        constraints = []
        for i in range(self.steps - 1):
            xk_start = i * self.numStates
            xk_end = xk_start + self.numStates
            xk_1_start = xk_end
            xk_1_end = xk_1_start+self.numStates
            uk_start = self.totalStates + i * self.numControls
            uk_end = uk_start + self.numControls
            uk_1_start = uk_end
            uk_1_end = uk_1_start + self.numControls
            # hk =  x[-1] / self.steps # time step into the future is
            const = lambda x : x[xk_1_start : xk_1_end] - x[xk_start : xk_end] - \
                               (0.5*(x[-1]/ self.steps)* (self.model(x[xk_1_start : xk_1_end], x[uk_1_start : uk_1_end]) +\
                                    self.model(x[xk_start : xk_end], x[uk_start : uk_end])))
            constraints.append(NonlinearConstraint(const, 0,0))
        return constraints
    '''
    set the position where the quad copter should be.
    '''
    def SetTargetPosition(self, currentState : np.array, targetState : np.array):
        # trajectory optimization problem.
        start_constraint = {'type': 'eq', 'fun': lambda x: x[0 : self.numStates]-currentState}
        terminal_constraint = {'type': 'eq', 'fun': lambda x: x[self.totalStates - self.numStates : self.totalStates]-targetState}
        total_constraints = (*self.collocationConstraints ,start_constraint, terminal_constraint)
        x0 = np.zeros(self.totalOptimizationVariables)
        x0[0 : self.numStates] = currentState
        x0[self.totalStates - self.numStates : self.totalStates] = targetState
        x0[-1] = 3 # assume time taken is 3
        timeToTrajectory = x0[-1]
        x0[self.totalStates : self.totalStates + self.totalControls] = np.ones(self.steps * self.numControls)*2*self.g

        for k in range(0, self.steps):
            x0[k * self.numStates:(k + 1)* self.numStates] = k / self.steps * (targetState - currentState) + currentState
            deviation = targetState - currentState
            x0[k * self.numStates] = np.clip(deviation[3]/timeToTrajectory, -1, 1)
            x0[k * self.numStates + 1] = np.clip(deviation[4]/timeToTrajectory, -1, 1)

        # res = minimize(self.objective, x0, method='SLSQP', bounds = self.bounds, constraints = total_constraints, options={'disp': True})
        # print(res)
        self.plotStates(x0)
        
    '''
    take the current state of the quad copter.
    the position , orientation, rotational and positional velocity.

    solve the MPC problem and return the next Force values 
    '''
    def GetControlSignal(self, currentState : np.array):

        '''
        this function takes the currentState and tries to optimize the controls
        along this trajectory.
        '''
        pass

    def plotStates(self,x):

        plt.figure("xvel")
        plt.plot(x[0:self.totalStates:self.numStates], 'go-', label='xvel', linewidth=2)
        plt.figure("yvel")
        plt.plot(x[1:self.totalStates:self.numStates], 'go-', label='yvel', linewidth=2)
        plt.figure("phivel")
        plt.plot(x[2:self.totalStates:self.numStates], 'go-', label='phivel', linewidth=2)
        plt.figure("x")
        plt.plot(x[3:self.totalStates:self.numStates], 'go-', label='x', linewidth=2)
        plt.figure("y")
        plt.plot(x[4:self.totalStates:self.numStates], 'go-', label='y', linewidth=2)
        plt.figure("phi")
        plt.plot(x[5:self.totalStates:self.numStates], 'go-', label='phi', linewidth=2)
        plt.figure("F1")
        plt.plot(x[self.totalStates:self.totalControls +self.totalStates:self.numControls], 'go-', label='F1', linewidth=2)
        plt.figure("F2")
        plt.plot(x[self.totalStates + 1:self.totalControls +self.totalStates:self.numControls], 'go-', label='F2', linewidth=2)
        plt.show()

    
