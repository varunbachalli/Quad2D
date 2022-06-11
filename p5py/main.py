from p5 import *
from UISetup import UIsetup
from Simulation.ButtonStateMachine import ButtonManager
windowWidth = 1080
windowHeight = 640
inputManager = None
outputManager = None
buttonManager = []
simManager = None

def setup():
    global inputManager, buttonManager, outputManager, simManager
    size(windowWidth, windowHeight)
    inputManager, buttonManager, outputManager, simManager = UIsetup(windowWidth , windowHeight)
    

def draw():
    global inputManager, buttonManager, outputManager , simManager
    background(204)
    inputManager.InputDisplay([mouse_x,mouse_y])
    simManager.UpdateSimulation()
    buttonManager.draw()
    outputManager.Plot()


def mouse_pressed():
    if(buttonManager.IsAnyButtonPressed([mouse_x,mouse_y])):
        return

    inputManager.SetPosition([mouse_x,mouse_y])


if __name__ == "__main__":
    run()