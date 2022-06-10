from abc import ABC, abstractmethod

class Plotter(ABC):
    def __init__(self):
        self.show = False

    @abstractmethod
    def plot(self):
        pass

    def togglePlot(self):
        self.show = not self.show
