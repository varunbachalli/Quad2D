# this file should contain the whole set up of the UI.
# create a function called setup() that creates all buttons , links all the functions to the buttons.
# and returns the SimulationManager, DisplayManager and InputManager. to be used in main.py
from p5 import *

from Simulation.InputManager import InputManager
from Simulation.DisplayManager import DisplayManager
from Simulation.SimulationManager import SimulationManager
from Simulation.Plotters.QuadPlotter import QuadPlotter
from Simulation.UIElements.Button import Button
from Simulation.UIElements.ButtonStack import ButtonStack
from Simulation.ButtonStateMachine import ButtonManager


def InputUISetup(canvasWidth, canvasHeight):
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

def SimulationSetup(inputManager, buttonManager):
    outputManager = DisplayManager()
    simManager = SimulationManager(inputManager, outputManager)
    #Quad Setup
    quadPlotter = QuadPlotter(inputManager.rect)
    outputManager.AddPlotter(quadPlotter)
    simManager.AssignQuadPlotter(quadPlotter)
    buttonManager.setRightButtonStackCallback([quadPlotter.togglePlot])
    buttonManager.setLeftButtonStackCallback([lambda: simManager.SetupSimulation(inputManager.startPosition, None)])

    return outputManager, simManager

def UIsetup(canvasWidth , canvasHeight):
    inputManager, buttonManager = InputUISetup(canvasWidth, canvasHeight)
    outputManager, simManager = SimulationSetup(inputManager, buttonManager)
    return inputManager, buttonManager, outputManager, simManager


    # drawing texts are computationally expensive for some reason.

