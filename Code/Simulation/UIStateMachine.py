'''
This file has the StateMachine that controls the whole flow of the entire application.

Inputs : Button clicks, Input Manager touches
States : Simulation Is Running, Simulation Is Complete, Simulation In Setup. 
Outputs : DisplayManager and Plotters, Buttons , Simulation


If Simulation Is In Setup: 
        Call To Action Button is Pressed : 
            If Simulation Is Running : 
                PauseSimulation
            Else:
                If Start and End are set : 
                    Then Setup The Simulation. 
                    Allow All the plots to be shown.
                    Hide All the Button Texts. 
                    Hide the CallToActionButton, Change the State of the  
                Else : 
                    Then Do Nothing.
                    Show All the texts and restore colors to white. 
                    Stop the Simulation and reset the quads positions and the controllers position.
            
        Input Manager Touches: 
            If Simulation Is Running 
                Stop Simulation
                Reset InputManager start and end positions.
                Show All the texts and buttons.
'''
from enum import Enum
from Simulation.InputManager import InputManager
from Simulation.DisplayManager import DisplayManager
from Simulation.Plotters.ControlPlotter import ControlPlotter
from Simulation.SimulationManager import SimulationManager
from Simulation.Plotters.QuadPlotter import QuadPlotter
from Simulation.Plotters.TrajectoryPlotter import TrajectoryPlotter
from Simulation.UIElements.Button import Button
from Simulation.UIElements.ButtonStack import ButtonStack
from Simulation.ButtonManager import ButtonManager

class ProgramState(Enum):
    Setup = 1
    Start = 2
    Pause = 3
    

class UIStateMachine:

    
    def __init__(self, canvasWidth, canvasHeight):
        self.canvasWidth = canvasWidth
        self.canvasHeight = canvasHeight    
        self.Setup()
        self.currentState = ProgramState.Setup

    def Setup(self):
        self.inputManager, self.buttonManager = self.InputUISetup(self.canvasWidth, self.canvasHeight)
        self.outputManager, self.simManager = self.SimulationSetup(self.inputManager, self.buttonManager)

    def InputUISetup(self,canvasWidth, canvasHeight):
        # see BaseUI.png for plan
        simulationCanvasWidth = canvasWidth // 2
        simulationCanvasHeight = canvasHeight
        simulationCanvasStartX = canvasWidth//4
        simulationCanvasStartY = 0

        inputManager = InputManager(simulationCanvasStartX,
                                    simulationCanvasStartY, 
                                    simulationCanvasWidth,
                                    simulationCanvasHeight)
        buttonManager = ButtonManager(canvasWidth, canvasHeight)

        return inputManager, buttonManager

    def SimulationSetup(self, inputManager : InputManager, buttonManager : ButtonManager):
        outputManager = DisplayManager()
        simManager = SimulationManager(inputManager, outputManager)
        #Quad Setup
        quadPlotter = QuadPlotter(inputManager.rect)
        outputManager.AddPlotter(quadPlotter)
        simManager.AssignQuadPlotter(quadPlotter)

        trajPlotter = TrajectoryPlotter(10, inputManager.rect)
        outputManager.AddPlotter(trajPlotter)
        simManager.AssignTrajectoryPlotter(trajPlotter)

        controlPlotter = ControlPlotter(inputManager.rect)
        outputManager.AddPlotter(controlPlotter)
        simManager.AssignControlPlotter(controlPlotter)

        buttonManager.setRightButtonStackCallback([quadPlotter.togglePlot, trajPlotter.togglePlot , controlPlotter.togglePlot])
        buttonManager.setLeftButtonStackCallback([self.CallToActionButtonPressed])

        return outputManager, simManager

    
    def draw(self, mouseX, mouseY):
        self.inputManager.InputDisplay([mouseX,mouseY])
        self.simManager.UpdateSimulation()
        self.buttonManager.draw()
        self.outputManager.Plot()

    def mousePressed(self, mouseX, mouseY):
        if(self.buttonManager.IsAnyButtonPressed([mouseX,mouseY])):
            return
        self.InputButtonPressed([mouseX,mouseY])

    def CallToActionButtonPressed(self):
        if(self.currentState == ProgramState.Setup):
            if(self.inputManager.startPosition is not None and\
               self.inputManager.endPosition is not None):

                self.simManager.SetupSimulation(self.inputManager.startPosition, self.inputManager.endPosition)
                self.currentState = ProgramState.Start
                self.StartActions()
            
        elif(self.currentState == ProgramState.Start):
            if(self.simManager.simRunning):
                self.simManager.PauseSimulation()
                self.currentState = ProgramState.Pause    
                self.PauseActions()        
        else:
            self.simManager.StartSimulation()
            self.currentState = ProgramState.Start
            self.StartActions()

    def SetupActions(self):
        self.inputManager.ResetStates()
        self.buttonManager.ShowAllButtonTexts() # shows all the texts and set colors to default
        self.outputManager.SetActive(False) # hide the outputs.
        
    def PauseActions(self):
        self.buttonManager.ShowAllButtonTexts()
        self.outputManager.SetActive(True)

    def StartActions(self):
        self.buttonManager.HideAllButtonTexts()
        self.outputManager.SetActive(True)

    def InputButtonPressed(self, mousePos):
        if(self.currentState == ProgramState.Setup):
            self.inputManager.SetPosition(mousePos)

        elif(self.currentState == ProgramState.Pause):
            self.currentState = ProgramState.Setup
            self.inputManager.SetPosition(mousePos)
            self.SetupActions()
        else:
            self.currentState = ProgramState.Setup
            self.inputManager.SetPosition(mousePos)
            self.SetupActions()
        