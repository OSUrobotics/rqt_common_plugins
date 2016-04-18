import random

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from PyQt4 import QtGui, QtCore

class RewardWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(RewardWidget, self).__init__(parent)
        self.samples = 0
        self.resize(1500, 100)

        self.figure = Figure()

        self.canvas = FigureCanvasQTAgg(self.figure)

        self.axes = self.figure.add_axes([0, 0, 1, 1])

        self.layoutVertical = QtGui.QVBoxLayout(self)
        self.layoutVertical.addWidget(self.canvas)

    def set_time_range(self, time_range):
    	self.time_range = time_range
    	range_length = int((time_range[1] - time_range[0]))
    	self.rewards = [0] * (range_length * 100)

    def add_data(self, data_range, data):
    	begin = int(round((data_range[0] - self.time_range[0]) * 100))
    	end = int(round((data_range[1] - self.time_range[0]) * 100)) + 1
    	self.rewards[begin:end] = [data for x in range(end-begin)]
    	

    	range_length = int((self.time_range[1] - self.time_range[0]))
    	self.axes.clear()
    	self.axes.set_xlim([0,range_length*100])
    	self.axes.set_ylim([-10.0,10.0])
        self.axes.plot(self.rewards)

        self.canvas.draw()
        #print(self.rewards)