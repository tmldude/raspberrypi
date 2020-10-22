from nav_algo.boat import BoatController
from nav_algo.physics_engine import PhysicsEngine
import nav_algo.boat as boat
from nav_algo.events import Events
from nav_algo.navigation_helper import *
import nav_algo.events as events
import sys
from PyQt5 import QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget, QSpinBox
import pyqtgraph as pg
import time
import numpy as np


class GUI:
    def __init__(self, boatController: boat.BoatController):
        self.boatController = boatController

        pg.setConfigOption('background', 'w')
        self.app = QtGui.QApplication(sys.argv)  # init Qt
        self.intro_w = QtGui.QWidget()  # top level widge to hold everything

        # exit cleaner
        exit_action = QtGui.QAction("Exit", self.app)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(lambda: exit(0))

        # Display the widget as a new window
        self.intro_w.setWindowTitle("CUSail Simulator")
        self.intro_layout = QtGui.QGridLayout()
        self.intro_w.setLayout(self.intro_layout)

        nav_button = QPushButton('Run Navigation Algorithm')
        self.intro_layout.addWidget(nav_button, 0, 0)
        nav_button.clicked.connect(self.setupNavAlgoMenu)
        event_button = QPushButton('Run An Event Algorithm')
        self.intro_layout.addWidget(event_button, 0, 1)
        event_button.clicked.connect(self.setupEventAlgoMenu)

        self.intro_w.show()

        self.app.exec_()  # start GUI in a new thread

    def setupEventAlgoMenu(self):
        self.intro_w.hide()

        self.event_w = QWidget()
        self.event_layout = QtGui.QGridLayout()
        self.event_w.setLayout(self.event_layout)
        self.event_w.setWindowTitle("Event Algorithm Simulator")

        # add a dropdown for event types
        self.event_label = QtGui.QLabel('Event Type:')
        self.event_layout.addWidget(self.event_label, 0, 0)
        self.event_selector = QtGui.QComboBox()
        self.events = Events.__iter__()
        for event in Events.__iter__():
            self.event_selector.addItem(str(event))
        self.event_layout.addWidget(self.event_selector, 0, 1)

        # add something to take in the mock sensor file name
        self.file_label = QtGui.QLabel('Mock Sensor Data File:')
        self.event_layout.addWidget(self.file_label, 1, 0)
        self.file_input = QtGui.QLineEdit()
        self.event_layout.addWidget(self.file_input, 1, 1)

        # add a button to call startEventAlgo
        self.event_button = QtGui.QPushButton('Start Event Algorithm')
        self.event_button.clicked.connect(self.startEventAlgo)
        self.event_layout.addWidget(self.event_button, 1, 2)

        # Testing: add a plot and some labels to see where the boat is
        self.plot = pg.PlotWidget()
        self.plot.showGrid(True, True, 0.3)
        self.plot.hideButtons()
        self.event_layout.addWidget(self.plot, 2, 0, 3, 3)
        self.waypoint_label = QtGui.QLabel('Target: (x, y)')
        self.event_layout.addWidget(self.waypoint_label, 5, 0)
        self.time_label = QtGui.QLabel('Time: ts')
        self.event_layout.addWidget(self.time_label, 5, 1)
        self.angles_label = QtGui.QLabel('Sail, Tail: s, t')
        self.event_layout.addWidget(self.angles_label, 5, 2)
        self.int_angle_label = QtGui.QLabel('Intended: d')
        self.event_layout.addWidget(self.int_angle_label, 6, 0)

        self.event_w.show()

    def setupNavAlgoMenu(self):
        self.intro_w.hide()

        self.nav_w = QWidget()
        self.nav_layout = QtGui.QGridLayout()
        self.nav_w.setLayout(self.nav_layout)
        self.nav_w.setWindowTitle("Navigation Algorithm Simulator")

        # widgets
        self.windDirLab = QtGui.QLabel('Wind Direction (wrt East):')
        self.windDirIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.nav_layout.addWidget(self.windDirLab, 0, 0)
        self.nav_layout.addWidget(self.windDirIn, 0, 1)

        self.windSpeedLab = QtGui.QLabel('Wind Speed (m/s):')
        self.windSpeedIn = QtGui.QSpinBox(maximum=100, minimum=0)
        self.nav_layout.addWidget(self.windSpeedLab, 1, 0)
        self.nav_layout.addWidget(self.windSpeedIn, 1, 1)

        self.rollLab = QtGui.QLabel('Roll:')
        self.rollIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.nav_layout.addWidget(self.rollLab, 2, 0)
        self.nav_layout.addWidget(self.rollIn, 2, 1)

        self.pitchLab = QtGui.QLabel('Pitch:')
        self.pitchIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.nav_layout.addWidget(self.pitchLab, 3, 0)
        self.nav_layout.addWidget(self.pitchIn, 3, 1)

        self.yawLab = QtGui.QLabel('Yaw (wrt East):')
        self.yawIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.nav_layout.addWidget(self.yawLab, 4, 0)
        self.nav_layout.addWidget(self.yawIn, 4, 1)

        self.targetLab = QtGui.QLabel('Target (x, y):')
        self.targetXIn = QtGui.QSpinBox(minimum=-1000)
        self.targetYIn = QtGui.QSpinBox(minimum=-1000)
        self.nav_layout.addWidget(self.targetLab, 5, 0)
        self.nav_layout.addWidget(self.targetXIn, 5, 1)
        self.nav_layout.addWidget(self.targetYIn, 5, 2)

        self.posLab = QtGui.QLabel('Position (x, y):')
        self.posXIn = QtGui.QSpinBox(minimum=-1000)
        self.posYIn = QtGui.QSpinBox(minimum=-1000)
        self.nav_layout.addWidget(self.posLab, 6, 0)
        self.nav_layout.addWidget(self.posXIn, 6, 1)
        self.nav_layout.addWidget(self.posYIn, 6, 2)

        self.calcButton = QtGui.QPushButton('Run Algorithm')
        self.nav_layout.addWidget(self.calcButton, 7, 0)
        self.calcButton.clicked.connect(self.runNavAlgo)

        self.intAngLab = QtGui.QLabel('Intended Angle: --')
        self.nav_layout.addWidget(self.intAngLab, 0, 4)

        self.sailAngLab = QtGui.QLabel('Sail Angle: --')
        self.nav_layout.addWidget(self.sailAngLab, 1, 4)

        self.tailAngLab = QtGui.QLabel('Tail Angle: --')
        self.nav_layout.addWidget(self.tailAngLab, 2, 4)

        self.nav_w.show()

    def runEventAlgo(self):
        # TODO mock the sensor readings
        # TODO call the nav helper navigate function to move one step forward
        # TODO pass boat config to physics engine, update gui
        # TODO use physics engine output to update boat position.
        self.phys_eng.moveOneStep()
        if self.phys_eng.current_waypoint is None:
            self.event_w.timer.stop()

        pen = pg.mkPen((100, 0, 255))
        brush = pg.mkBrush((100, 0, 255))
        self.plot.plot([self.phys_eng.params.com_pos.x],
                       [self.phys_eng.params.com_pos.y],
                       symbolPen=pen,
                       symbolBrush=brush,
                       symbol="o")
        self.time_label.setText('Time: {:.1f}s'.format(self.phys_eng.time))
        self.waypoint_label.setText('Target: ({:.2f}, {:.2f})'.format(
            self.phys_eng.current_waypoint.x,
            self.phys_eng.current_waypoint.y))
        self.angles_label.setText('Sail, Tail: {:.0f}, {:.0f}'.format(
            self.phys_eng.params.theta_s, self.phys_eng.params.theta_r))
        self.int_angle_label.setText('Intended: {:.2f}'.format(
            self.phys_eng.intended_angle))

    def startEventAlgo(self):
        # TODO get type of event from the dropdown menu
        # TODO get name of mock sensor file (maybe from a text box on gui)
        # TODO call the event function in nav helper
        # TODO call runEventAlgo every 0.1s(?) - use a QTimer

        # TODO reads in data from the file, doesn't do anything with it yet
        # try:
        #     file = open(self.file_input.text(), 'r')
        # except FileNotFoundError:
        #     print("file '" + self.file_input.text() + "' could not be found")

        # event_name = file.readline().split(": ")[1]

        # start = file.readline().split(": ")
        # start_coords = start[1].split(", ")
        # start_x = float(start_coords[0])
        # start_y = float(start_coords[1])

        # wind = file.readline().split(", ")
        # wind_spd = float(wind[0].split(": ")[1])
        # wind_dir = float(wind[1].split(": ")[1])

        # file_waypoints = []
        # for line in file:
        #     pt = line.split(", ")
        #     v = coord.Vector(x=float(pt[0]), y=float(pt[1]))
        #     file_waypoints.append(v)

        # TODO everything below here is just for testing
        boat_controller = BoatController(simulation=True)
        boat_controller.sensors.position = coord.Vector(x=0, y=0)
        boat_controller.sensors.yaw = 45.0
        boat_controller.sensors.wind_direction = 25.0
        boat_controller.sensors.wind_speed = 5.0

        waypoints = [
            coord.Vector(x=35, y=35),
            coord.Vector(x=55, y=35),
            coord.Vector(x=55, y=20),
            coord.Vector(x=35, y=20)
        ]

        self.phys_eng = PhysicsEngine(boat_controller, waypoints)

        self.event = Events.STATION_KEEPING

        self.event_w.timer = QTimer()
        self.event_w.timer.setInterval(10)
        self.event_w.timer.timeout.connect(self.runEventAlgo)
        self.event_w.timer.start()

    def runNavAlgo(self):
        self.boatController.sensors.position = coord.Vector(
            x=self.posXIn.value(), y=self.posYIn.value())
        self.waypoint = coord.Vector(x=self.targetXIn.value(),
                                     y=self.targetYIn.value())

        self.boatController.sensors.wind_direction = self.windDirIn.value()
        self.boatController.sensors.wind_speed = self.windSpeedIn.value()

        self.boatController.sensors.roll = self.rollIn.value()
        self.boatController.sensors.pitch = self.pitchIn.value()
        self.boatController.sensors.yaw = self.yawIn.value()

        intended_angle = newSailingAngle(self.boatController, self.waypoint)
        sail_angle, tail_angle = self.boatController.getServoAngles(
            intended_angle)

        self.intAngLab.setText('Intended Angle: ' + str(intended_angle))
        self.sailAngLab.setText('Sail Angle: ' + str(sail_angle))
        self.tailAngLab.setText('Tail Angle: ' + str(tail_angle))
