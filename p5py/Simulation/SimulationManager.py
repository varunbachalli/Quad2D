from .Plotters.QuadPlotter import QuadPlotter
from .InputManager import InputManager
from .DisplayManager import DisplayManager
from .MathModels.QuadCopter2D import QuadCopter2D
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

    
    def __init__(self, inputManager : InputManager,
                       displayManager : DisplayManager):
        self.inputManager = inputManager
        self.displayManager = displayManager
        self.quadCopter = QuadCopter2D(scalep5ToGlobal(inputManager.quadSize[0]),
                                       mass = 1)# mass = 1kg

    def StartSimulation(self):
        # here write code to initialize trajectory planner.
        # then initialize the high level controller for performing operations in parallel thread. 

        # Note: the low level controller should just use the next input in the sequence to carry 
        # out it's process if the MPC controller isn't done yet. 
        # the MPC controller should run on a seperate thread to avoid stalling the simulation.
        # use multiprocessing.Process from the tutorials. 

        # then initialize the low Level controller to take outputs from the high level controller
        # then take the inputs from the low level controller and set it to the quad to use as required.
        # simulate the quad and then call self.displayManager.Plot() 
        pass

    def PauseSimulation(self):
        pass

    def StopSimulation(self):
        pass

    
    def AssignQuadPlotter(self, plotter : QuadPlotter):
        self.quadCopter.SetPlotter(plotter)