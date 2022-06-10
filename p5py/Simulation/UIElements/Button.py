from .ButtonUtils import Rect

# define button in local coordinates
class Button:
    '''
    if(parent is column)
    then position is along y and size is along y in p5 coordinates
    '''
    def __init__(self,  position : float,size : float, parent):
        parentTopLeft = parent.rect.tl
        parentBottomRight = parent.rect.br
        if(parent.column):
            topLeft = [parentTopLeft[0], parentTopLeft[1] + position]
            bottomRight = [parentBottomRight[0], parentTopLeft[1] + position + size]

        else:
            topLeft = [parentTopLeft[0] + position, parentTopLeft[1]]
            bottomRight = [parentTopLeft[0] + position + size, parentBottomRight[1]]

        self.rect = Rect(topLeft,bottomRight)

        self.active = True
        self.text = "Custom Button"
        self.function = None

    '''
    sets the lambda to be called on click
    '''
    def setOnClick(self, function ):
        if(not callable(function)):
            function = lambda x : print(function)
        self.function = function

    '''
    runs the click function
    '''
    def click(self, param = None):
        if(not self.active):
            return

        if(self.function is None):
            print("No onclick")
            return

        try:
            if(param is None):
                self.function()
            else:
                self.function(param)
        except:
            print("function doesn't take this type of!")

    '''
    returns if the pointer is inside the button area
    '''
    def IsPointerInside(self, pointerPosition):
        return self.rect.IsPointInRect(pointerPosition)

    def setActive(self, active):
        self.active = active

    def setText(self, string):
        self.text = string

    def drawButton(self):
        if(not self.active):
            return

        self.rect.draw()

    def drawText(self):
        if(not self.active):
            return

        if(self.text is not None):
            self.rect.drawText(self.text)
