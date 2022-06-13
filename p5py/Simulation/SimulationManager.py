from .Plotters.QuadPlotter import QuadPlotter
from .Plotters.TrajectoryPlotter import TrajectoryPlotter
from .InputManager import InputManager
from .DisplayManager import DisplayManager
from .MathModels.QuadCopter2D import QuadCopter2D
from .MathModels.TrajectoryPlanner import DynamicOptimization_Trajectory
from .Utilities.UIUtils import scalep5ToGlobal

'''
this class is the orchastrator between the input, display managers and the various simulation components.
'''
class SimulationManager:
    '''
    this function sets up all the controllers, quad and the trajectory planner.
    it creates a plotter for each of the objects and then assigns them to the object.
    each object is meant to set values in the plotter after the calculation is complete.
    '''
    def SetupSimulation(self, startPosition , endPosition):
        self.quadCopter.setStartPosition(startPosition)
        endState = [0,0,0,endPosition[0], endPosition[1], 0]
        self.trajectoryPlanner.SetTrajectory(self.quadCopter.states,endState)
        self.simSetup = True
    
    def __init__(self, inputManager : InputManager,
                       displayManager : DisplayManager):
        self.inputManager = inputManager
        self.displayManager = displayManager
        self.quadCopter = QuadCopter2D(scalep5ToGlobal(inputManager.quadSize[0]),
                                       mass = 1)# mass = 1kg
        self.trajectoryPlanner = DynamicOptimization_Trajectory(self.quadCopter, 50)

        self.simSetup = False

    def UpdateSimulation(self):
        if(not self.simSetup):
            return

        if(self.quadCopter is None):
            return

        if(self.quadCopter.get_position()[1] < 1.5):
            self.quadCopter.update_position([3*9.81/2, 3*9.81/2])
        else:
            self.quadCopter.update_position([0,0])

        # here write code to initialize trajectory planner.
        # then initialize the high level controller for performing operations in parallel thread. 

        # Note: the low level controller should just use the next input in the sequence to carry 
        # out it's process if the MPC controller isn't done yet. 
        # the MPC controller should run on a seperate thread to avoid stalling the simulation.
        # use multiprocessing.Process from the tutorials. 

        # then initialize the low Level controller to take outputs from the high level controller
        # then take the inputs from the low level controller and set it to the quad to use as required.
        # simulate the quad and then call self.displayManager.Plot() 

    def PauseSimulation(self):
        pass

    def StopSimulation(self):
        pass

    
    def AssignQuadPlotter(self, plotter : QuadPlotter):
        if(self.quadCopter is not None ):
            self.quadCopter.SetPlotter(plotter)

    def AssignTrajectoryPlotter(self, plotter : TrajectoryPlotter): 
        if(self.trajectoryPlanner is not None ):
            self.trajectoryPlanner.SetPlotter(plotter)
