from .Plotters.APlotter import Plotter

'''
description: 
    takes a bunch of plotters from the simulation and then displays the plots for each of them.
'''
class DisplayManager:
    def __init__(self):
        self.plotters = []
        self.active = True
    def Plot(self):
        for plotter in self.plotters:
            if(plotter is not None and self.active):
                plotter.plot()

    def AddPlotter(self, plotter : Plotter):
        self.plotters.append(plotter)

    def SetActive(self, active):
        self.active = active
    