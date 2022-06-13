from p5 import *
import numpy as np
class Rect:
    def __init__(self, tl, br):
        self.tl = tl
        self.br = br
        self.size = [br[0] - tl[0] , br[1] - tl[1]]
        self.position = np.array([(self.tl[0] + self.br[0])/2, (self.tl[1] + self.br[1])/2])
        self.angle = 0
        self.rect_theta = np.arctan2(*self.size) #self.size[1]/self.size[0] 
        self.rect_R = np.sqrt(self.size[0]**2 + self.size[1]**2)/2
        self.quadAngles = [self.rect_theta, np.pi - self.rect_theta, np.pi + self.rect_theta , -self.rect_theta]
    def IsPointInRect(self, point):
        return self.tl[0] < point[0] < self.br[0] and self.tl[1] < point[1] < self.br[1]
    
    def draw(self, fill_ = None):
        if(fill_ is not None):
            fill(*fill_)
        if(self.angle == 0):
            rect(self.tl[0] , self.tl[1] , self.size[0] , self.size[1])
        else:
            quad(*self.quad_positions)

    def drawText(self, string : str) -> None: 
        text(string, ((self.tl[0] + self.br[0])/2,(self.tl[1] + self.br[1])/2))

    def updateCenter(self, position):
        self.position = np.array(position)
        self.tl = [position[0] - self.size[0]/2 , position[1] - self.size[1]/2]
        self.br = [position[0] + self.size[0]/2 , position[1] + self.size[1]/2]


    def updateAngle(self, angle):
        self.angle = angle
        self.quad_positions = [self.position + np.array([-np.sin(q+ angle) , np.cos(q + angle)])*self.rect_R for q in self.quadAngles]
        self.quad_positions =np.ravel(self.quad_positions)