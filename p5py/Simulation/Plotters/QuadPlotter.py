from .APlotter import Plotter
from Simulation.UIElements.ButtonUtils import Rect
from ..Utilities.UIUtils import transformPositionToP5
from ..constants import simquadColor
class QuadPlotter(Plotter):
    def __init__(self, parentRect : Rect):
        super().__init__()
        self.parentRect = parentRect
        self.position = transformPositionToP5(parentRect, [1, 6])
        self.size = [100,20]
        self.rect = Rect([self.position[0] - self.size[0]/2, self.position[1] - self.size[1]/2],
                         [self.position[0] + self.size[0]/2, self.position[1] + self.size[1]/2])
        
    '''
    implementation of abstract method.
    '''
    def plot(self):
        if(self.show):
            self.rect.draw(simquadColor)

    def setPosition(self, position):
        position = transformPositionToP5(self.parentRect, position)
        self.rect.updateCenter(position)

    def setAngle(self,angle):
        self.angle = angle
        self.rect.updateAngle(angle)