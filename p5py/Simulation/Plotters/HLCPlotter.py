from .APlotter import Plotter

class HighLevelControllerPlotter(Plotter):

    def __init__(self):
        super().__init__()
    '''
    implementation of abstract method.
    '''
    def plot(self):
        if(self.show):
            pass