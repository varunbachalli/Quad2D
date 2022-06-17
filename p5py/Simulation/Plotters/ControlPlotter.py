from .APlotter import Plotter
from Simulation.UIElements.ButtonUtils import Rect
from ..constants import controlquadColor
from ..Utilities.UIUtils import transformPositionToP5

class ControlPlotter(Plotter):

    def __init__(self, parentRect : Rect):
        super().__init__()
        self.parentRect = parentRect
        self.size = [100,20]
        self.rects = None
        self.points = None
        
    '''
    implementation of abstract method.
    '''
    def plot(self):
        if(self.show and self.rects is not None):
            for rect in self.rects:
                rect.draw(controlquadColor)

    
    def setTrajectory(self, points):
        self.firstSetup(points)
        for i, p in enumerate(points):
            self.points[i] = transformPositionToP5(self.parentRect, p[:2])
            self.rects[i].updateCenter(self.points[i])
            self.rects[i].updateAngle(p[-1])

    def firstSetup(self, points):
        if(self.points is None or self.rects is None):
            self.points = [[0,0]]*len(points)
            self.rects = [Rect([pos[0] - self.size[0]/2, pos[1] - self.size[1]/2],
                            [pos[0] + self.size[0]/2, pos[1] + self.size[1]/2]) for pos in self.points]