import enum 
from .UIElements.ButtonUtils import Rect
from .UIElements.Button import Button
from .Utilities.UIUtils import transformPositionToGlobal


'''
takes the input from the screen and passes the relevant information to the simulation.
'''
class InputManager:
    def __init__(self,windowStartX, windowStartY, windowWidth, windowHeight):
        self.rect = Rect([windowStartX, windowStartY], 
                         [windowStartX + windowWidth, windowStartY + windowHeight])
        self.startPosition = None
        self.endPosition = None
        self.startRect = None
        self.endRect = None
        self.quadSize = [100,20]
        self.currentRect = Rect([windowStartX, windowStartY],[windowStartX + self.quadSize[0], windowStartY + self.quadSize[1]])
        self.active = True

    def SetActive(self, active) : self.active = active
    def GetRect(self) : return self.rect
    '''
    checks if a point is inside the rect
    '''
    def PointIsInsideWindow(self, point): return self.rect.IsPointInRect(point)

    '''
    this function takes the input from the canvas and sets the start position of the quad.
    '''
    def setStart(self, position):
        if(not self.PointIsInsideWindow(position)):
            return
        self.startPosition = transformPositionToGlobal(self.rect, position)
        self.startRect = Rect(position,[position[0] + self.quadSize[0], position[1] + self.quadSize[1]])
        self.startRect.updateCenter(position)

    '''
    this function takes the position 
    '''
    def setEnd(self, position):
        if(not self.PointIsInsideWindow(position)):
            return
        self.endPosition = transformPositionToGlobal(self.rect, position)
        self.endRect = Rect(position,[position[0] + self.quadSize[0], position[1] + self.quadSize[1]])
        self.endRect.updateCenter(position)

    
    def InputDisplay(self,mousePosition):
        if(not self.active):
            return
        if(self.startRect is not None): 
            self.startRect.draw([255,0,0])
        if(self.endPosition is not None): 
            self.endRect.draw([0,255,0])
        if(self.endPosition is None):
            if(self.PointIsInsideWindow(mousePosition)):
                self.currentRect.updateCenter(mousePosition)
                self.currentRect.draw([0,127])
    
    def SetPosition(self,position):
        if(self.startPosition is None or self.endPosition is not None):
            self.setStart(position)
            self.endPosition = None
            self.endRect = None
        else:
          self.setEnd(position)
    


    def ResetStates(self):
        self.startPosition = None
        self.endPosition = None
        self.startRect = None
        self.endRect = None