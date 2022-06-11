from .ButtonUtils import Rect
from .Button import Button

class ButtonStack:
    '''
    if column then position is along x and size is width in p5 coordinates
    '''
    def __init__(self, position : float,
                       size : float,
                       canvasSize, # [canvasWidth, canvasHeight]
                       column = True):
        self.position = position
        self.size = size
        self.column = column
        self.canvasSize = canvasSize
        if(self.column):
            topLeft = [position, 0]
            bottomRight = [position + size, canvasSize[1]]
            self.rect = Rect(topLeft, bottomRight)
        else:
            topLeft = [0, position]
            bottomRight = [canvasSize[0],position + size]
            self.rect = Rect(topLeft, bottomRight)

        self._buttons = []
    
    def IsPointerInside(self, pointerPosition):
        if(self.rect.IsPointInRect(pointerPosition)):
            for button in self._buttons:
                if(button.IsPointerInside(pointerPosition)):
                    return True, button
            return True , None
        return False, None

    def setActive(self, active):
        self.active = active
        for button in self._buttons:
            button.setActive(active)

    def Addbutton(self, button : Button):
        self._buttons.append(button)
    
    def drawButtons(self):
        [button.drawButton() for button in self._buttons]

    def drawButtonTexts(self):
        [button.drawText() for button in self._buttons]

    def CreateNButtons(self, N):
        buttons = []
        size = self.canvasSize[1]/N if self.column else self.canvasSize[0]/N
        for i in range(N):
            button = Button(i*size, size, self)
            buttons.append(button)

        return buttons

    def __getitem__(self, key):
        if(key >= len(self._buttons)):
            key = 0
        return self._buttons[key]

    def __len__(self):
        return len(self._buttons)