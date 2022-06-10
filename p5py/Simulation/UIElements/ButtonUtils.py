from p5 import *

class Rect:
    def __init__(self, tl, br):
        self.tl = tl
        self.br = br
        self.size = [br[0] - tl[0] , br[1] - tl[1]]
    
    def IsPointInRect(self, point):
        return self.tl[0] < point[0] < self.br[0] and self.tl[1] < point[1] < self.br[1]
    
    def draw(self, fill_ = None):
        if(fill_ is not None):
            fill(*fill_)

        rect(self.tl[0] , self.tl[1] , self.size[0] , self.size[1])

    def drawText(self, string : str) -> None: 
        text(string, ((self.tl[0] + self.br[0])/2,(self.tl[1] + self.br[1])/2))

    def updateCenter(self, position):
        self.tl = [position[0] - self.size[0]/2 , position[1] - self.size[1]/2]
        self.br = [position[0] + self.size[0]/2 , position[1] + self.size[1]/2]

