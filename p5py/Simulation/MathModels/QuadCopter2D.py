from sysconfig import get_python_version
import numpy as np
from ..Plotters.QuadPlotter import QuadPlotter
'''
Quadcopter in 2d
'''
class QuadCopter2D:
    '''
    starting position is in the 0 , 0 in the xz axis by default.
    and with pitch of 0
    0.1 length across by default.
    '''
    def __init__(self,l = 0.1,
                      mass = 1.0):
        self.l = l
        self.position = np.zeros(2)
        self.theta = 0
        self.mass = mass
        self.inertia = mass*(l**2)/12
        self.states = np.zeros(6)  # velocities, positions
        self.numControls = 2 
        self.plotter = None

    
    def setStartPosition(self, start_position = np.zeros(2),
                               start_orientation = 0.0):
        self.position = start_position
        self.theta = start_orientation
        self.states[3] = start_position[0] 
        self.states[4] = start_position[1]
        self.states[5] = start_orientation
                            
    '''

    states are 
    [dx, dy, dtheta, x, y, theta]
    controls are
    [F1, F2]

    statespace equation 
    d/dt[x]T = [(u[0] + u[1])/m * sin(x[5]),
                (u[0] + u[1])/m * cos(x[5]) - g,
                (u[1] - u[2])/(l * I)  
                x[0],
                x[1],
                x[2]
    function that updates the position of the quad-copter using runge kutta integration of the system
    of equations as shown in the derivation. 

    applyTorque applies different torques at the left and right rotors
    delT is the simulation time.

    update using euler method.
    '''
    # def update_position(self, applyMotorTorque = np.zeros(2), delT= 1e-3):
    #     '''
    #     system of equations for simplified model of pendulum.
    #     '''
    #     pass

    def update_position(self, applyForce = np.zeros(2), delT = 1e-2):
        '''
        system of equations 
        ddx = (F1 + F2)/m * sin(theta)
        ddy = (F1 + F2)/m * cos(theta) - g
        ddtheta = (F2 - F1)/(l * I)  
        '''
        if(self.numControls!= len(applyForce)):
            print("Error! more controls provided than available")
        F1 = applyForce[0]
        F2 = applyForce[1]
        previous_state = np.copy(self.states)

        self.states[0] = delT * ((F1 + F2) * np.sin(previous_state[5])/self.mass) + previous_state[0] # dx
        self.states[1] = delT * (((F1 + F2) * np.cos(previous_state[5])/self.mass) - 9.81) + previous_state[1] #dy
        self.states[2] = delT * ((F1 - F2)/ (self.l * self.inertia)) + previous_state[2] #dtheta

        self.states[3] = (delT * previous_state[0])+ previous_state[3] # x
        self.states[4] = (delT * previous_state[1])+ previous_state[4] # y 
        self.states[5] = (delT * previous_state[2])+ previous_state[5] # theta

        print("Position Updated!!")
        if(self.plotter is not None):
            self.plotter.setPosition(self.get_position())
    '''
    returns the global position of the quadcopter.
    '''
    def get_position(self):
        return self.states[3:5]

    '''
    returns the global pitch angle of the quadcopter.
    '''
    def get_orientation(self):
        return self.states[5]


    def get_rotation_matrix(self):
        theta = self.get_orientation()
        return np.array([[np.cos(theta), np.sin(theta)],
                         [-np.sin(theta),np.cos(theta)]])
    '''
    generates random drift in the x z axis.
    '''
    def getDrift(self):
        return np.zeros(2)
    

    def SetPlotter(self, plotter : QuadPlotter):
        self.plotter = plotter
        self.plotter.setPosition(self.get_position())
    
    def getPlot(self):
        rotationMatrix = self.get_rotation_matrix()
        positionleft = self.get_position() + np.ravel(np.matmul(rotationMatrix, np.array([self.l/2, 0])))
        positionright = self.get_position() + np.ravel(np.matmul(rotationMatrix, -np.array([self.l/2, 0])))
        return [positionleft[0], positionright[0]],\
               [positionleft[1], positionright[1]]
        