from .Plotters.QuadPlotter import QuadPlotter
from .Plotters.TrajectoryPlotter import TrajectoryPlotter
from .Plotters.ControlPlotter import ControlPlotter
from .InputManager import InputManager
from .DisplayManager import DisplayManager
from .MathModels.QuadCopter2D import QuadCopter2D
from .MathModels.TrajectoryPlanner import DynamicOptimization_Trajectory
from .MathModels.MPC_TrajectoryFollower import MPC_TrajectoryFollower
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
        self.currentTime = 0.0
        self.simSetup = True
        self.StartSimulation()
        
    
    def __init__(self, inputManager : InputManager,
                       displayManager : DisplayManager):
        self.inputManager = inputManager
        self.displayManager = displayManager
        self.quadCopter = QuadCopter2D(scalep5ToGlobal(inputManager.quadSize[0]),
                                       mass = 1)# mass = 1kg
        self.trajectoryPlanner = DynamicOptimization_Trajectory(self.quadCopter, 50)

        self.controller = MPC_TrajectoryFollower(self.quadCopter,10,delT= 0.1)
        self.simSetup = False
        self.quadPlotter = None
        self.trajPlotter = None
        self.controlPlotter = None
        self.simRunning = False

    def UpdateSimulation(self):
        if(not self.simSetup or not self.simRunning):
            return

        if(self.quadCopter is None):
            return


        controls = self.controller.GetControlInput(self.quadCopter.states, self.trajectoryPlanner, self.currentTime)
        self.quadCopter.update_position(controls)
        self.currentTime += 0.01

    def StartSimulation(self):
        self.simRunning = True

    def PauseSimulation(self):
        self.simRunning = False

    def StopSimulation(self):
        self.simSetup = False

    
    def AssignQuadPlotter(self, plotter : QuadPlotter):
        if(self.quadCopter is not None ):
            self.quadCopter.SetPlotter(plotter)
            self.quadPlotter = plotter

    def AssignTrajectoryPlotter(self, plotter : TrajectoryPlotter): 
        if(self.trajectoryPlanner is not None ):
            self.trajectoryPlanner.SetPlotter(plotter)
            self.trajPlotter = plotter

    def AssignControlPlotter(self, plotter : ControlPlotter):
        if(self.controller is not None):
            self.controller.SetPlotter(plotter)
            self.controlPlotter = plotter

    def SetActive(self, active) : 
        if(self.trajPlotter is not None): self.trajPlotter.togglePlot(active)
        if(self.quadPlotter is not None): self.quadPlotter.togglePlot(active)
        if(self.controlPlotter is not None): self.controlPlotter.togglePlot(active)