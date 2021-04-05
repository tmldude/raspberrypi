import nav_algo.servo as servo
import nav_algo.sensor_array as sens_arr
import nav_algo.sim_sensors as sim_sens
import nav_algo.coordinates as coord
import numpy as np


class BoatController:
    def __init__(self, coordinate_system=None, simulation=False):
        self.sim = simulation
        if simulation:
            self.sensors = sim_sens.sensorSim()
        else:
            self.coordinate_system = coordinate_system
            self.sensors = sens_arr.sensorArray()

        # servo angles
        self.sail_angle = 0
        self.tail_angle = 0

    def getPosition(self):
        if self.sim:
            return self.sensors.position
        return coord.Vector(self.coordinate_system, self.sensors.latitude,
                            self.sensors.longitude)

    def updateSensors(self):
        self.sensors.readAll()

    def getServoAngles(self, intended_angle: float):
        # intended angle is the target angle wrt global coordinates
        curr_heading = self.sensors.yaw
        curr_alpha = curr_heading - self.sensors.wind_direction
        int_alpha = intended_angle - self.sensors.wind_direction

        print('current {}, intended {}'.format(curr_heading, intended_angle))

        if np.sign(curr_alpha) != np.sign(int_alpha):
            sail = 0
            if np.sign(int_alpha) > 0:
                # turning from starboard to port
                tail = 30
            else:
                # turning from port to starboard
                tail = -30

            print('setting sail {}, tail {}'.format(sail, tail))
            return sail, tail

        # else set the angle based on the points of sail
        tail = 0
        sail = -1 * int_alpha / 2

        print('setting sail {}, tail {}'.format(sail, tail))
        return sail, tail

    def setServos(self, intended_angle: float):
        sail, tail = self.getServoAngles(intended_angle)

        # set the servos
        self.tail_angle = servo.setTail(coord.rangeAngle(tail))
        self.sail_angle = servo.setSail(coord.rangeAngle(sail))