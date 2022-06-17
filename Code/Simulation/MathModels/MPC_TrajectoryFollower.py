import numpy as np
from casadi import *
import matplotlib.pyplot as plt
from .TrajectoryPlanner import DynamicOptimization_Trajectory
from .QuadCopter2D import QuadCopter2D
from ..Plotters.ControlPlotter import ControlPlotter
class MPC_TrajectoryFollower:
    def __init__(self, quad : QuadCopter2D, steps = 10, delT = 1e-2):
        self.l = quad.l
        self.mass = quad.mass
        self.inertia = quad.inertia
        self.currentState = quad.states 
        self.numStates = len(quad.states)
        self.numControls = quad.numControls 
        self.N = steps # number of intermediate steps
        self.totalStates = self.N * self.numStates
        self.totalOptimizationVariables = self.N * self.numStates + self.numControls * (self.N - 1) # total time
        self.totalControls = self.numControls * self.N
        self.g = 9.81
        self.delT = delT
        self.T = self.N*delT
        self.posBounds = (-np.inf, np.inf)
        self.angleBounds = (-np.pi/3, np.pi/3)
        self.velBounds = (-2,2)
        self.rotVelBounds = (-np.pi/12, np.pi/12)
        self.controlBounds = (-10*self.g,10*self.g)
        self.model, self.cost = self.CreateCostAndModel()
        self.modelIntegration, self.costIntengration = self.GetIntegrator(self.model, self.cost)
        self.nlp = self.SetUpNLP(self.modelIntegration, self.costIntengration)
        self.plotter = None


    def CreateCostAndModel(self):
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

        xdot_  = vertcat(xdot, ydot, phidot) 
        W = dot(xdot_, xdot_) + 10*dot(u_, u_)

        f = Function('f', [x_, u_], [ddx],['x_k','F'],['f_k'])
        cost = Function("W", [x_ , u_], [W], ["x_k" , 'F'], ["J"])
        return f, cost


    '''
    runge kutta integrator.
    '''
    def GetIntegrator(self, model, cost):
        X0 = MX.sym('X0', 6)
        U = MX.sym('U', 2)
        # here U is assumed to be constant over the entire interval
        
        k1 = model(x_k = X0, F = U)['f_k']
        k2 = model(x_k = X0 + self.delT/2 * k1, F = U)['f_k']
        k3 = model(x_k = X0 + self.delT/2 * k2, F = U)['f_k']
        k4 = model(x_k = X0 + self.delT/2 * k3, F = U)['f_k']
        X = X0 + self.delT/6*(k1 +2*k2 +2*k3 +k4)

        k1_q = cost(x_k = X0, F = U)['J']
        k2_q = cost(x_k = X0 + self.delT/2 * k1, F = U)['J']
        k3_q = cost(x_k = X0 + self.delT/2 * k2, F = U)['J']
        k4_q = cost(x_k = X0 + self.delT/2 * k3, F = U)['J']

        Q = self.delT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        modelIntegration = Function('F', [X0, U], [X],['xk','uk'],['xk_1'])
        costIntegration = Function("J", [X0, U], [Q], ['xk', 'uk'], ['Jk'])
        return modelIntegration, costIntegration

    def SetUpNLP(self, modelIntegrator, costIntegrator):
        Xk = MX.sym('X_0', 6)

        w = [Xk]
        w0 = [0]*6
        lbw = [self.velBounds[0],
               self.velBounds[0],
               self.rotVelBounds[0],
               self.posBounds[0],
               self.posBounds[0],
               self.angleBounds[0]]
        ubw = [self.velBounds[1],
               self.velBounds[1],
               self.rotVelBounds[1],
               self.posBounds[1],
               self.posBounds[1],
               self.angleBounds[1]]

        J = 0
        g = []
        lbg = []
        ubg = []
        
        trackedStates =[]
        trackedStates_0 = []
        trackedStateslbw= []
        trackedStatesubw = []
        for i in range(self.N - 1):
            Fk = MX.sym(f'F_{i}', 2)
            w += [Fk]
            w0 +=  [self.g/2, self.g/2]
            lbw += [self.controlBounds[0], self.controlBounds[0]]
            ubw += [self.controlBounds[1], self.controlBounds[1]]

            X_trac = MX.sym(f"X_track_{i}", 6)
            trackedStates.append(X_trac)
            trackedStates_0 += [0]*6 # these will be initialized before the problem is solved
            trackedStateslbw += [0]*6
            trackedStatesubw +=[0]*6

            # cost function integration.
            J += costIntegrator(xk = Xk, uk = Fk)['Jk'] + 50 * dot(Xk - X_trac , Xk - X_trac)
            
            # model function integration
            f_x_k = modelIntegrator(xk=Xk, uk = Fk)['xk_1']

            Xk_1 = MX.sym(f'X_{i+1}', 6)
            w+= [Xk_1]
            w0 += [0]*6
            lbw += [self.velBounds[0],
                    self.velBounds[0],
                    self.rotVelBounds[0],
                    self.posBounds[0],
                    self.posBounds[0],
                    self.angleBounds[0]]
            ubw += [self.velBounds[1],
                    self.velBounds[1],
                    self.rotVelBounds[1],
                    self.posBounds[1],
                    self.posBounds[1],
                    self.angleBounds[1]]

            # system dynamics.
            g += [f_x_k - Xk_1] 
            lbg += [0]*6
            ubg += [0]*6
            
            Xk = Xk_1

        X_trac = MX.sym(f"X_track_{self.N}", 6)
        trackedStates.append(X_trac)
        trackedStates_0 += [0]*6 # these will be initialized before the problem is solved
        trackedStateslbw += [0]*6
        trackedStatesubw+=[0]*6
        J += 50 * dot(Xk - X_trac , Xk - X_trac)  # terminal state should also converge

        nlp_init = {
            "w" : w + trackedStates,
            "g" : g,
            "J" : J,
            "w0" : w0 + trackedStates_0,
            "lbg" : lbg,
            "ubg" : ubg,
            "ubw" : ubw + trackedStatesubw,
            "lbw" : lbw + trackedStateslbw
        }

        return nlp_init

    def GetControlInput(self, startState, trajectoryOptimizer, time):
        w0 = self.nlp['w0']
        lbw = self.nlp['lbw']
        ubw = self.nlp['ubw']
        lbg = self.nlp['lbg']
        ubg = self.nlp['ubg']

        w0[:self.numStates] = startState
        lbw[:self.numStates] = startState
        ubw[:self.numStates] = startState

        trackedStates = []
        for i in range(self.N):
            trackedStates+= np.ravel(trajectoryOptimizer.GetXAtTime(time + i * self.delT)).tolist()

        w0[self.totalOptimizationVariables:] = trackedStates
        lbw[self.totalOptimizationVariables:] = trackedStates
        ubw[self.totalOptimizationVariables:] = trackedStates
        
        cost = self.nlp["J"]

        prob = {'f': cost, 'x': vertcat(*self.nlp["w"]), 'g' : vertcat(*self.nlp['g'])}
        solver = nlpsol('solver', 'ipopt', prob)
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
        
        self.w_opt = sol['x'].full().flatten()

        if(self.plotter is not None):
            x = self.w_opt[3:self.totalOptimizationVariables:self.numStates + self.numControls]
            y = self.w_opt[4:self.totalOptimizationVariables:self.numStates + self.numControls]
            phi = self.w_opt[5:self.totalOptimizationVariables:self.numStates + self.numControls]
            traj = list(zip(x,y,phi))
            self.plotter.setTrajectory(traj)

        return self.w_opt[self.numStates: self.numStates+ self.numControls]

    def SetPlotter(self, plotter : ControlPlotter):
        self.plotter = plotter
    
if __name__ == "__main__":
    q = QuadCopter2D(l = 1.5, start_orientation=0, start_position=np.ones(2))
    c = MPC_TrajectoryFollower(q,steps = 100, delT= 0.01)
    target = np.zeros_like(q.states)
    target[3:5] = [4,4]
    traj = DynamicOptimization_Trajectory(q, 50)
    traj.SetTrajectory(q.states, target)
    for i in range(30):
        control = c.GetControlInput(startState = q.states,
                                    trajectoryOptimizer = traj, 
                                    time = i * 0.01)
        q.update_position(control, 0.1)
        plt.show()
        