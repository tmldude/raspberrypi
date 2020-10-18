import nav_algo.coordinates as coord
import nav_algo.boat
from nav_algo.navigation_helper import *
import numpy as np
import scipy.interpolate
import pandas as pd


class PhysicsEngine():
    def __init__(self, boat_controller, waypoints):
        self.getAlphas()

        self.boat_controller = boat_controller
        self.waypoints = waypoints
        self.current_waypoint = waypoints.pop(0)

        self.params = self.SimParams()
        self.params.com_pos = boat_controller.getPosition()
        self.params.v_wind = coord.Vector(
            angle=boat_controller.sensors.wind_direction)
        self.params.v_wind.scale(boat_controller.sensors.wind_speed)
        self.params.theta_b = boat_controller.sensors.yaw

        self.timestep = 0.1  # seconds
        self.time = 0
        self.idx = 0

        self.z = np.array([
            self.params.com_pos.x, self.params.com_pos.y, self.params.v_boat.x,
            self.params.v_boat.y, self.params.theta_b, self.params.omega,
            self.params.theta_r, self.params.theta_s
        ])

    def moveOneStep(self):
        # run the nav algo every 2 mock seconds
        if self.time % 2 == 0:
            print("did this")
            self.mockNavAlgo()

        self.z = np.array([
            self.params.com_pos.x, self.params.com_pos.y, self.params.v_boat.x,
            self.params.v_boat.y, self.params.theta_b, self.params.omega,
            self.params.theta_r, self.params.theta_s
        ])

        zdot = self.dynamics()
        self.z = np.add(zdot, np.multiply(self.z, self.timestep))
        self.params.com_pos.x = self.z[0]
        self.params.com_pos.y = self.z[1]
        self.params.v_boat.x = self.z[2]
        self.params.v_boat.y = self.z[3]
        self.params.theta_b = self.z[4]
        self.params.omega = self.z[5]
        self.params.theta_r = self.z[6]
        self.params.theta_s = self.z[7]

        self.idx += 1
        self.time = self.idx * self.timestep

    def mockNavAlgo(self):
        # TODO detection radius
        if self.params.com_pos.xyDist(self.current_waypoint) < 15.0:
            if len(self.waypoints) > 0:
                self.current_waypoint = self.waypoints.pop(0)
            else:
                self.current_waypoint = None
                return

        intended_angle = newSailingAngle(self.boat_controller,
                                         self.current_waypoint)
        sail_angle, tail_angle = self.boat_controller.getServoAngles(
            intended_angle)

        self.params.theta_s = sail_angle + self.params.theta_b
        self.params.theta_r = tail_angle + sail_angle + self.params.theta_b

    def dynamics(self):
        xdot = np.transpose(self.z[2:4])
        thetadot = np.transpose(self.z[5])
        thetaboat = self.z[4]
        vr = np.array([self.params.v_wind.x, self.params.v_wind.y])
        vrn = np.divide(vr, np.linalg.norm(vr))

        if np.linalg.norm(xdot) == 0:
            xnorm = np.array([0, 0])
        else:
            xnorm = np.divide(xdot, np.linalg.norm(xdot))

        thetaruddot = self.params.theta_r
        thetasaildot = self.params.theta_s

        alpha = self.alpha_cl_cd[:, 0]
        cl = self.alpha_cl_cd[:, 1]
        cd = self.alpha_cl_cd[:, 2]

        _, alpha_s, alpha_r, alpha_k = self.computeAlphas(vr, xdot)

        # interpolate data for exact alpha
        CLr = scipy.interpolate.pchip_interpolate(alpha, cl, np.abs(alpha_r))
        CDr = scipy.interpolate.pchip_interpolate(alpha, cd, np.abs(alpha_r))
        CLs = scipy.interpolate.pchip_interpolate(alpha, cl, np.abs(alpha_s))
        CDs = scipy.interpolate.pchip_interpolate(alpha, cd, np.abs(alpha_s))
        CLk = scipy.interpolate.pchip_interpolate(alpha, cl, np.abs(alpha_k))
        CDk = scipy.interpolate.pchip_interpolate(alpha, cd, np.abs(alpha_k))
        CDh = 6.5  # based on Jesse Miller's experiments
        Cdamph = 2  # based on Jesse Miller's experiments

        vv = np.array([vrn[0], vrn[1], 0])
        kk = np.array([0, 0, 1])
        L = np.cross(vv, kk)  # direction of lift (air)

        vk = np.array([xnorm[0], xnorm[1], 0])
        K = np.cross(vk, kk)  # direction of lift water

        # Force calculation (NOTE: water->xdot(boat velocity), air->vr(boat and wind velocity)
        f_lr = np.multiply(
            CLr * self.params.area_rudder * .5 * self.params.rho_air *
            (np.linalg.norm(vr))**2, L)
        f_lr = f_lr[0:2]

        f_lk = np.multiply(
            -CLk * self.params.area_keel * .5 * self.params.rho_water *
            (np.linalg.norm(xdot))**2, K)
        f_lk = f_lk[0:2]

        f_ls = np.multiply(
            CLs * self.params.area_sail * .5 * self.params.rho_air *
            (np.linalg.norm(vr))**2, L)
        f_ls = f_ls[0:2]

        f_ds = np.multiply(
            CDs * self.params.area_sail * .5 * self.params.rho_air *
            (np.linalg.norm(vr))**2, vrn)

        f_dk = np.multiply(
            -CDk * self.params.area_keel * .5 * self.params.rho_water *
            (np.linalg.norm(xdot))**2, xnorm)

        f_dr = np.multiply(
            CDr * self.params.area_rudder * .5 * self.params.rho_air *
            (np.linalg.norm(vr))**2, vrn)

        f_dh = np.multiply(-CDh * (np.linalg.norm(xdot))**2, xdot)

        # assign velocity derivative
        vdot = 1 / self.params.boat_mass * (f_ls + f_lk + f_lr + f_ds + f_dr +
                                            f_dk + f_dh)

        # Moment Calculation
        r_r = np.array([
            np.cos(np.deg2rad(thetaboat + self.params.theta_s)) * (-.2286),
            np.sin(np.deg2rad(thetaboat + self.params.theta_s)) * (-.2286)
        ])

        r_k = np.array([0, 0])
        r_s = np.array([0, 0])

        f_r = np.add(f_lr, f_dr)
        m_r = np.cross(np.array([r_r[0], r_r[1], 0]),
                       np.array([f_r[0], f_r[1], 0]))

        f_k = np.add(f_lk, f_dk)
        m_k = np.cross(np.array([r_k[0], r_k[1], 0]),
                       np.array([f_k[0], f_k[1], 0]))

        f_s = np.add(f_ls, f_ds)
        m_s = np.cross(np.array([r_s[0], r_s[1], 0]),
                       np.array([f_s[0], f_s[1], 0]))

        m_h = np.multiply(-Cdamph, np.array([0, 0, thetadot * np.pi / 180.0]))

        sumM = np.add(m_r, m_k)
        sumM = np.add(sumM, m_s)
        sumM = np.add(sumM, m_h)
        sumMM = sumM[2]

        omegadot = 180.0 / np.pi * (1.0 / self.params.inertia * sumMM)

        zdot = np.array([
            xdot[0], xdot[1], vdot[0], vdot[1], thetadot, omegadot,
            thetaruddot, thetasaildot
        ])

        return zdot

    def getAlphas(self):
        old_data = np.array(
            pd.read_csv('nav_algo/simData/alphas.csv',
                        delim_whitespace=True,
                        header=None))
        a = old_data[:, 1]
        cl = np.multiply(1.2, old_data[:, 2])
        cd = old_data[:, 3]

        new_data = np.stack((a, cl, cd), axis=1)

        # add symmetric values above 180 degrees
        new_a = np.subtract(360.0, a[1:-1])
        new_cl = np.multiply(-1.0, cl[1:-1])
        new_cd = cd[1:-1]

        new_rows = np.stack((new_a, new_cl, new_cd), axis=1)
        new_data = np.concatenate((new_data, new_rows), axis=0)

        idx = np.argsort(new_data[:, 0])
        self.alpha_cl_cd = new_data[idx]

    def computeAlphas(self, vr, xdot):
        self.theta_wind_rel = 180.0 / np.pi * np.arctan2(vr[1], vr[0])
        self.theta_boat_vel = 180.0 / np.pi * np.arctan2(xdot[1], xdot[0])

        balph_boat = self.params.theta_b - (self.theta_wind_rel + 180)
        balph_s = balph_boat + self.params.theta_s
        balph_r = balph_s + self.params.theta_r
        balph_k = self.params.theta_b - self.theta_boat_vel

        alph_boat = coord.rangeAngle(balph_boat)
        alph_r = coord.rangeAngle(balph_r)
        alph_s = coord.rangeAngle(balph_s)
        alph_k = coord.rangeAngle(balph_k)

        return alph_boat, alph_s, alph_r, alph_k

    class SimParams():
        # SI Units (kg, m, s)
        rho_air = 1.225  # density of air
        rho_water = 1000  # density of water

        # TODO
        boat_mass = 10
        hull_length = 1
        hull_width = .3
        sail_length = 0.2032
        sail_width = 0.1
        keel_length = 0.076
        keel_width = 0.05
        rudder_length = 0.15
        rudder_width = 0.05

        # TODO largest cross section areas
        area_sail = 0.899 * 0.2
        area_keel = 0.33 * 0.07
        area_rudder = 0.533 * 0.1524

        # TODO inertia of the boat
        inertia = boat_mass * (hull_length**2 + hull_width**2) / 20.0

        # center of mass of the boat
        com_pos = coord.Vector(x=0, y=0)

        # angle of the boat wrt x-axis
        theta_b = 0

        # boat and wind speeds
        v_boat = coord.Vector(x=0, y=0)
        v_wind = coord.Vector(x=0, y=-5)

        # angles of rudder and sail wrt boat centerline
        theta_s = 0
        theta_r = 0

        # total angular velocity
        omega = 0