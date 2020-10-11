import nav_algo.coordinates as coord


class Engine():
    def __init__(self):
        self.tail_sail_area = 10.0  # TODO
        self.main_sail_area = 20.0  # TODO
        self.keel_area = 30.0  # TODO

        self.air_density = 1.225  # kg/m^3
        self.water_density = 1000  # kg/m^3

        self.water_speed = 1  # TODO

    def getVelocity(self, wind_dir, wind_speed, sail_angle, tail_angle, yaw):
        self.wind_dir = wind_dir
        self.wind_speed = wind_speed
        # TODO sail angle is given wrt boat frame, not global frame
        self.sail_angle = sail_angle if sail_angle < 180 else -sail_angle
        self.alpha = wind_dir - self.sail_angle - yaw
        self.alpha_tail = tail_angle  # TODO

        F_main = self.mainSailLift().vectorAdd(self.mainSailDrag())
        F_tail = self.tailSailLift().vectorAdd(self.tailSailDrag())
        F_keel = self.keelLift().vectorAdd(self.keelDrag())

        F_sail = F_main.vectorAdd(F_tail)
        F_net = F_sail.vectorAdd(F_keel)
        return F_net

    def mainSailLift(self):
        coef_lift = 1.0  # TODO get this for a NACA 0018 airfoil
        mag = 0.5 * coef_lift * self.air_density * self.main_sail_area * (
            self.wind_speed**2)
        F_lift = coord.Vector(angle=self.wind_dir +
                              90 if self.alpha > 0 else self.wind_dir -
                              90)  # TODO check logic
        return F_lift.scale(mag)

    def mainSailDrag(self):
        coef_drag = 1.0  # TODO get this for a NACA 0018 airfoil
        mag = 0.5 * coef_drag * self.air_density * self.main_sail_area * (
            self.wind_speed**2)
        F_drag = coord.Vector(angle=self.wind_dir)  # TODO check logic
        return F_drag.scale(mag)

    def tailSailLift(self):
        coef_lift = 1.0  # TODO get this for the tail airfoil
        mag = 0.5 * coef_lift * self.air_density * self.tail_sail_area * (
            self.wind_speed**2)
        F_lift = coord.Vector(angle=self.wind_dir +
                              90 if self.alpha_tail > 0 else self.wind_dir -
                              90)  # TODO check logic
        return F_lift.scale(mag)

    def tailSailDrag(self):
        coef_drag = 1.0  # TODO get this for tail airfoil
        mag = 0.5 * coef_drag * self.air_density * self.tail_sail_area * (
            self.wind_speed**2)
        F_drag = coord.Vector(angle=self.wind_dir +
                              90 if self.alpha_tail > 0 else self.wind_dir -
                              90)  # TODO check logic
        return F_drag.scale(mag)

    def keelLift(self):
        coef_lift = 1.0  # TODO get this for keel
        mag = 0.5 * coef_lift * self.water_density * self.keel_area * (
            self.water_speed**2)
        F_lift = coord.Vector(angle=self.wind_dir +
                              90 if self.alpha > 0 else self.wind_dir -
                              90)  # TODO check logic
        return F_lift.scale(mag)

    def keelDrag(self):
        coef_drag = 1.0  # TODO get this for keel
        mag = 0.5 * coef_drag * self.water_density * self.keel_area * (
            self.water_speed**2)
        F_drag = coord.Vector(angle=self.wind_dir +
                              90 if self.alpha > 0 else self.wind_dir -
                              90)  # TODO check logic
        return F_drag.scale(mag)