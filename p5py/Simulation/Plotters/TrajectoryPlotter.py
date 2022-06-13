from .APlotter import Plotter
from Simulation.UIElements.ButtonUtils import Rect
from ..Utilities.UIUtils import transformPositionToP5
from p5 import *

class TrajectoryPlotter(Plotter):

    def __init__(self, numPoints, parentRect : Rect):
        super().__init__()
        self.parentRect = parentRect
        self.size = [100,20]
        self.numPoints = numPoints
        self.points = [[0,0]]*self.numPoints
        self.rects = [Rect([pos[0] - self.size[0]/2, pos[1] - self.size[1]/2],
                         [pos[0] + self.size[0]/2, pos[1] + self.size[1]/2]) for pos in self.points]
        self.setTrajectory([[0,0,0]]*self.numPoints)

        
    '''
    implementation of abstract method.
    '''
    def plot(self):
        if(self.show):
            for rect in self.rects:
                rect.draw([0,150,150,150])
            self.drawTrajectory()
            
    def setTrajectory(self, points):
        for i, p in enumerate(points):
            if(i < self.numPoints):
                self.points[i] = transformPositionToP5(self.parentRect, p[:2])
                self.rects[i].updateCenter(self.points[i])
                self.rects[i].updateAngle(p[-1])

        self.points = np.array(self.points)
        z = np.polyfit(self.points[:,0], self.points[:,1], 5)
        self.polynom = np.poly1d(z)
        self.minX = np.min(self.points[:,0])
        self.maxX = np.max(self.points[:,0])

    def drawTrajectory(self):
        if(self.polynom is None or self.minX is None or self.maxX is None):
            return
        
        polyPoints = np.array([[x,self.polynom(x)] for x in np.linspace(self.minX, self.maxX, 100)])
        polyLines = zip(polyPoints[:-1], polyPoints[1:])
        for p1, p2 in polyLines:
            line(*p1, *p2)
