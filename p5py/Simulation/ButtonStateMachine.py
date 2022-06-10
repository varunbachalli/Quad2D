'''
this file decides the states of the buttons in the scene.
'''
from p5 import *
from .UIElements.ButtonStack import ButtonStack

class ButtonManager:
    def buttonsSetup(self):
        font = create_font("Arial.ttf", 16) 
        text_font(font)
        text_align("CENTER")


    def __init__(self, canvasWidth, canvasHeight):
        self.buttonsSetup()
        
        buttonStacks = []
        leftButtonStack = ButtonStack(0, canvasWidth//4, [canvasWidth , canvasHeight])
        buttons = leftButtonStack.CreateNButtons(1)

        leftButtonStack.Addbutton(buttons[0])
        buttons[0].setText("Call\nTo\nAction")
        buttons[0].setOnClick(lambda : print("Call To Action Pressed"))

        buttonStacks.append(leftButtonStack)

        rightButtonStack = ButtonStack(3 * canvasWidth//4, canvasWidth//4, [canvasWidth , canvasHeight])

        buttons = rightButtonStack.CreateNButtons(3)
        buttons[0].setText("Show\nTrajectory")
        buttons[0].setOnClick(lambda : print("Show Trajectory Pressed"))

        buttons[1].setText("Show\nLow Level\nControls")
        buttons[1].setOnClick(lambda : print("Show Low Level Controls Pressed"))

        buttons[2].setText("Show\nMotion\nPlanning")
        buttons[2].setOnClick(lambda : print("Show Motion Planning"))

        for button in buttons:
            rightButtonStack.Addbutton(button)

        buttonStacks.append(rightButtonStack)

        self.leftButtonStack = leftButtonStack
        self.rightButtonStack = rightButtonStack
        self.buttonStacks = buttonStacks

    def draw(self):
        fill(255)
        [buttonStack.drawButtons() for buttonStack in self.buttonStacks]
        fill(0)
        [buttonStack.drawButtonTexts() for buttonStack in self.buttonStacks] 

    def IsAnyButtonPressed(self, mousePosition):
        for buttonStack in self.buttonStacks:
            inside, button = buttonStack.IsPointerInside(mousePosition)
            if(inside):
                if(button is not None):
                    button.click()
                return True
        return False