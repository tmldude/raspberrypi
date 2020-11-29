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
        # TODO check logic for all of this, I'm 99% sure it's wrong - CM
        # angle_of_attack = 15.0
        # if self.sensors.wind_direction < 180.0:
        #     angle_of_attack = -15.0

        # offset = self.sensors.yaw - intended_angle
        # tail = round(self.sensors.wind_direction + offset)
        # sail = round(tail + angle_of_attack)

        # # convert sail and tail from wrt global to wrt boat
        # tail = tail - self.sensors.yaw
        # sail = sail - self.sensors.yaw

        # tail = coord.rangeAngle(tail)
        # sail = coord.rangeAngle(sail)

        # print('{} {}'.format(sail, tail))

        sail = 0
        tail = 0

        port = True

        offset = intended_angle - self.sensors.wind_direction
        cur_offset = self.sensors.yaw - self.sensors.wind_direction
        print("intended heading wrt wind {}, current {}".format(
            offset, cur_offset))
        if offset < 0:
            print("starboard")
            port = False
        else:
            print("port")

        if np.abs(cur_offset) < 21:
            print('currently stuck in irons')

        #points of sail - these are servo angles (sail wrt boat and tail wrt mainsail)
        if np.abs(offset) < 20.0:
            print("stuck in irons :(")
            # try to break out, deal with the rest later
            tail = -30 if port else 30
        elif np.abs(offset) < 45.0:
            sail = -15 if port else 15
            tail = -15 if port else 15
            print("close hauled")
        elif np.abs(offset) < 75.0:
            sail = -30 if port else 30
            tail = -15 if port else 15
            print("close reach")
        elif np.abs(offset) < 115.0:
            sail = -45 if port else 45
            tail = -15 if port else 15
            print("beam reach")
        elif np.abs(offset) < 145.0:
            sail = -60 if port else 60
            tail = -15 if port else 15
            print("broad reach")
        elif np.abs(offset) < 160.0:
            print("training run -- can't hit")
        else:
            print("run -- can't hit")

        sail = self.sensors.wind_direction if cur_offset <= 0 else -1.0 * self.sensors.wind_direction
        if abs(cur_offset) < 5:
            tail = 0
        else:
            tail = 30 if cur_offset <= 0 else -30

        sail = 30
        tail = 15

        return sail, tail

    def setServos(self, intended_angle: float):
        sail, tail = self.getServoAngles(intended_angle)

        # set the servos
        self.tail_angle = servo.setTail(coord.rangeAngle(tail))
        self.sail_angle = servo.setSail(coord.rangeAngle(sail))