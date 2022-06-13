import numpy as np
from casadi import *
from .QuadCopter2D import QuadCopter2D
import matplotlib.pyplot as plt

from Simulation.Plotters.TrajectoryPlotter import TrajectoryPlotter

class DynamicOptimization_Trajectory:
    '''
    used to get the quad parameters like I, l ,m
    '''
    def __init__(self, quad : QuadCopter2D, steps = 10):
        self.l = DM(quad.l)
        self.mass = DM(quad.mass)
        self.inertia = DM(quad.inertia)
        self.currentState = quad.states 
        self.numStates = len(quad.states)
        self.numControls = quad.numControls 
        self.state_trajectory = []
        self.N = steps# number of intermediate steps
        self.totalStates = self.N * self.numStates
        self.totalOptimizationVariables = self.N * (self.numControls + self.numStates) +  1 # total time
        self.totalControls = self.numControls * self.N
        self.g = 9.81
        
        self.T = MX.sym("T") # final time trajectory
        
        self.posBounds = (-np.inf, np.inf)
        self.angleBounds = (-np.pi/4, np.pi/4)
        self.velBounds = (-2,2)
        self.timeBounds = (0, np.inf)
        self.rotVelBounds = (-np.pi/24, np.pi/24)
        self.controlBounds = ((-10*self.g,10*self.g), (-10*self.g,10*self.g))

        self.model, self.cost = self.GetCollocationConstraintsAndCost()
        self.solver , self.nlpinit = self.SetupNLP()
    
    def GetCollocationConstraintsAndCost(self):
        xdot = MX.sym("xdot")
        ydot = MX.sym("ydot")
        phidot = MX.sym("phidot")
        
        x = MX.sym("x")
        y = MX.sym("y")
        phi = MX.sym("phi")

        F1 = MX.sym("F1")
        F2 = MX.sym("F2")

        x_ = vertcat(xdot, ydot, phidot, x, y, phi)
        u_ = vertcat(F1, F2)
        ddx = vertcat((F1 + F2)/self.mass * np.sin(phi), 
                      (F1 + F2)/self.mass * np.cos(phi) - self.g,
                      (F1 - F2)/(self.l * self.inertia),
                      xdot,
                      ydot,
                      phidot)

        # W = xdot * xdot + ydot * ydot + 5*phidot * phidot #+ F1 * F1 + F2 * F2
        W =  F1 * F1 + F2 * F2 + 10* ydot * ydot + 10*phidot * phidot
        f = Function('f', [x_, u_], [ddx],['x_k','F'],['f_k'])
        cost = Function("W", [x_ , u_], [W], ["x_k" , 'F'], ["wk"])
        return f, cost

    def SetupNLP(self):
        Xk = MX.sym('X_0', 6)
        Fk = MX.sym('F_0', 2)
        w0 = [0]*6 + [9.81/2]*2 
        w = [Xk, Fk]
        J = 0
        g = []
        lbg = []
        ubg = []
        for i in range(self.N - 1):
            Xk_1 = MX.sym(f'X_{i+1}', 6)
            Fk_1 = MX.sym(f'F_{i+1}', 2)
            hk = self.T/self.N
            J += 0.5 * hk * (self.cost(x_k = Xk, F = Fk)['wk'] + self.cost(x_k = Xk_1, F= Fk_1)['wk'])
            g += [Xk_1-Xk - 0.5*hk *(self.model(x_k=Xk, F= Fk)['f_k'] + self.model(x_k=Xk_1, F= Fk_1)['f_k'])]
            lbg += [0]*6
            ubg += [0]*6
            Xk = Xk_1
            Fk = Fk_1
            w += [Xk, Fk]
            w0 += [0]*6 + [9.81/2]*2 
        

        lbw = self.N *[self.velBounds[0],
               self.velBounds[0],
               self.rotVelBounds[0],
               self.posBounds[0],
               self.posBounds[0],
               self.angleBounds[0],
               self.controlBounds[0][0],
               self.controlBounds[1][0]]

        ubw = self.N * [self.velBounds[1],
               self.velBounds[1],
               self.rotVelBounds[1],
               self.posBounds[1],
               self.posBounds[1],
               self.angleBounds[1],
               self.controlBounds[0][1],
               self.controlBounds[1][1]]

        w += [self.T]
        w0 += [5]
        lbw += [1E-3] # but it's atleast 0.001 seconds
        ubw += [float('inf')] # time is unbounded upwards
        nlp_init = {
            "w0" : w0,
            "lbw" : lbw,
            "ubw" : ubw ,
            "lbg" : lbg,
            "ubg" : ubg
        }

        prob = {'f': J + 100 * self.T, 'x': vertcat(*w), 'g': vertcat(*g)}
        solver = nlpsol('solver', 'ipopt', prob)
        return solver, nlp_init

    def GetXAtTime(self, t):
        if(self.splines == None):
            return [0]*self.numStates

        if(t <= 0):
            t = 0
            spline = self.splines[0]

        if(t >= self.totalTime):
            t = self.totalTime
            spline = self.splines[-1]
        else:
            splineNumber = int(t/self.totalTime * (self.N-1))
            spline = self.splines[splineNumber]
        
        delK = t - spline['t_k']
        x_k = spline['x_k']
        f_k = spline['f_k']
        gamma_k = spline['gamma_k']
        return x_k + delK * f_k + (delK**2) * gamma_k 

    def InterpolateStates(self, w):
        xdot = w[0:-1:self.numStates + self.numControls]
        ydot = w[1:-1:self.numStates + self.numControls]
        phidot =w[2:-1:self.numStates + self.numControls]
        x = w[3:-1:self.numStates + self.numControls]
        y = w[4:-1:self.numStates + self.numControls]
        phi = w[5:-1:self.numStates + self.numControls]
        F1 = w[6:-1:self.numStates + self.numControls]
        F2 = w[7:-1:self.numStates + self.numControls]
        self.totalTime = w[-1]
        steps = self.N
        hk = self.totalTime/steps
        # create a bunch of functions for time 
        self.splines = []
        for i in range(steps - 1):
            xk =  np.array([xdot[i],ydot[i],phidot[i],x[i],y[i], phi[i]])
            Fk = np.array([F1[i], F2[i]]) 
            xk_1 = np.array([xdot[i + 1],ydot[i + 1],phidot[i + 1],x[i + 1],y[i + 1], phi[i + 1]])
            Fk_1 =np.array([F1[i + 1], F2[i + 1]]) 
            fk = self.model(x_k = xk,F = Fk)['f_k']
            fk_1 = self.model(x_k = xk_1, F = Fk_1)['f_k']
            gamma_k = -1/(2*hk) * (fk - fk_1)
            self.splines.append({
                    "x_k" : xk,
                    "f_k" : fk,
                    "t_k" : i * self.totalTime/(steps - 1),
                    "gamma_k"  : gamma_k
            })
        
    def SetTrajectory(self, currentState, targetState):
        if(currentState is None or targetState is None):
            return
        # reset w0 reset lbw and ubw
        w0 = self.nlpinit['w0']
        lbw = self.nlpinit['lbw']
        ubw = self.nlpinit['ubw']
        lbg = self.nlpinit['lbg']
        ubg = self.nlpinit['ubg']

        w0[:self.numStates] = currentState
        lbw[:self.numStates] = currentState
        ubw[:self.numStates] = currentState

        finalStateStart = self.totalOptimizationVariables - 1 - self.numControls - self.numStates # total - controls - states - Time
        finalStateEnd = self.totalOptimizationVariables - 1 - self.numControls

        w0[finalStateStart:finalStateEnd] = targetState
        lbw[finalStateStart:finalStateEnd] = targetState
        ubw[finalStateStart:finalStateEnd] = targetState
        
        sol = self.solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
        w_opt = sol['x'].full().flatten()
        self.w_opt = w_opt
        self.InterpolateStates(self.w_opt)
        if(self.plotter is not None):
            numPoints =self.plotter.numPoints
            hk = self.totalTime/numPoints
            traj = []
            for i in range(numPoints):
                x = self.GetXAtTime(i* hk)
                traj.append(x[3:6])
            self.plotter.setTrajectory(traj)

    def SetPlotter(self, plotter : TrajectoryPlotter):
        self.plotter = plotter


    

if __name__ == "__main__":
    q = QuadCopter2D(l = 1.5, start_orientation=0, start_position=np.ones(2))
    t = DynamicOptimization_Trajectory(q, 100)
    target = np.zeros_like(q.states)
    target[3:5] = [4,4]
    t.SetTrajectory(q.states, target)
    interpolatedStates = []
    interpolatedTime = np.linspace(0, t.totalTime, 250)
    for time in interpolatedTime:
        states = t.GetXAtTime(time)
        interpolatedStates.append(np.array(states))
    interpolatedStates = np.array(interpolatedStates)
    t.Plots(t.w_opt, interpolatedStates,interpolatedTime)
    plt.show()
