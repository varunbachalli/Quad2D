from .APlotter import Plotter

class TrajectoryPlotter(Plotter):

    def __init__(self):
        super().__init__()
    '''
    implementation of abstract method.
    '''
    def plot(self):
        if(self.show):
            pass