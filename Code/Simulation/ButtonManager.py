'''
this file decides the states of the buttons in the scene.
'''
from p5 import *
from sys import platform
from .UIElements.ButtonStack import ButtonStack
from .constants import simquadColor, trajquadColor, controlquadColor
class ButtonManager:
    def buttonsSetup(self):
        if platform == "darwin":
            font = create_font("Arial.ttf", 16) 
            text_font(font)
            text_align("CENTER")
            # OS X
    def __init__(self, canvasWidth, canvasHeight):
        self.buttonsSetup()
        
        buttonStacks = []
        leftButtonStack = ButtonStack(0, canvasWidth//4, [canvasWidth , canvasHeight])
        buttons = leftButtonStack.CreateNButtons(1)

        leftButtonStack.Addbutton(buttons[0])
        buttons[0].setText("Play!")

        buttonStacks.append(leftButtonStack)

        rightButtonStack = ButtonStack(3 * canvasWidth//4, canvasWidth//4, [canvasWidth , canvasHeight])

        buttons = rightButtonStack.CreateNButtons(3)
        buttons[0].setText("Show\nQuad")
        buttons[1].setText("Show\nTrajectory")
        buttons[2].setText("Show\nControl")

        for button in buttons:
            rightButtonStack.Addbutton(button)

        buttonStacks.append(rightButtonStack)

        self.leftButtonStack = leftButtonStack
        self.rightButtonStack = rightButtonStack
        self.buttonStacks = buttonStacks

    def draw(self):
        fill(255)
        [buttonStack.drawButtons() for buttonStack in self.buttonStacks]
        if platform == "darwin":
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
    

    def __setButtonStackCallback(self, buttonStack : ButtonStack, callbacks):
        for i, callback in enumerate(callbacks):
            if(i < len(buttonStack)):
                buttonStack[i].setOnClick(callback)
        
    def setLeftButtonStackCallback(self, callbacks):
        self.__setButtonStackCallback(self.leftButtonStack, callbacks)
    
    def setRightButtonStackCallback(self, callbacks):
        self.__setButtonStackCallback(self.rightButtonStack, callbacks)


    def HideAllButtonTexts(self):
        self.rightButtonStack.ShowText(False)
        self.leftButtonStack.ShowText(False)
        self.rightButtonStack[0].setButtonColor(simquadColor)
        self.rightButtonStack[1].setButtonColor(trajquadColor)
        self.rightButtonStack[2].setButtonColor(controlquadColor)
        self.leftButtonStack[0].setButtonColor([0,255,0,255])

    
    def ShowAllButtonTexts(self):
        self.rightButtonStack.ShowText(True)
        self.leftButtonStack.ShowText(True)
        self.rightButtonStack[0].setButtonColor(None)
        self.rightButtonStack[1].setButtonColor(None)
        self.rightButtonStack[2].setButtonColor(None)
        self.leftButtonStack[0].setButtonColor(None)