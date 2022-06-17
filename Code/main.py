from p5 import *
from Simulation.UIStateMachine import UIStateMachine
windowWidth = 1080
windowHeight = 640
states = UIStateMachine(windowWidth, windowHeight)

def setup():
    global states
    size(windowWidth, windowHeight)
    
def draw():
    global states
    background(204)
    states.draw(mouse_x, mouse_y)

def mouse_pressed():
    states.mousePressed(mouse_x, mouse_y)


if __name__ == "__main__":
    run()