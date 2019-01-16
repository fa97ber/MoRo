import numpy as np
from PoseEstimator.PlotUtilities import *
import random


class ParticleFilterPoseEstimator:

    def __init__(self):
        self.Particles = None
        self.robot = None


    def setRobot(self, robot):
        self.robot = robot
        return


    def getParticles(self):
        return self.Particles


    # 3a: Erzeugt n zufällige Partikel in dem vorgegebenen Posen-Bereich.
    def initialize(self, poseFrom, poseTo, n=200):
        xmin, ymin, thetamin = poseFrom
        xmax, ymax, thetamax = poseTo
        poseList = []
        for _ in range(n):
            ranx = np.random.uniform(xmin, xmax)
            rany = np.random.uniform(ymin, ymax)
            rantheta = np.random.uniform(thetamin, thetamax)
            pose = [ranx, rany, rantheta]
            poseList.append(pose)


        self.Particles = poseList
        return


    # 3b: Wendet auf alle Partikel den Bewegungsbefehl motion mit einem zufälligen Rauschen an.
    def integrateMovement(self, motion):

        T = 0.1

        # Motion noise parameter:
        k_d = 0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
        k_theta = (5.0 * 5.0/360.0) * (pi/180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
        k_drift = (2.0 * 2.0)/1.0 * (pi/180.0)**2  # drift noise parameter = 2deg*2deg / 1m

        maxSpeed = 1.0  # maximum speed
        maxOmega = np.pi  # maximum rotational speed

        v = motion[0]
        omega = motion[1]
        poseList = []

        # translational and rotational speed is limited:
        if omega > maxOmega:
            omega = maxOmega
        if omega < -maxOmega:
            omega = -maxOmega
        if v > maxSpeed:
            v = maxSpeed
        if v < -maxSpeed:
            v = -maxSpeed

        for pose in self.Particles:

            # Add noise to v:
            sigma_v_2 = (k_d / T) * abs(v)
            v_noisy = v + random.gauss(0.0, sqrt(sigma_v_2))

            # Add noise to omega:
            sigma_omega_tr_2 = (k_theta / T) * abs(omega)  # turning rate noise
            sigma_omega_drift_2 = (k_drift / T) * abs(v) # drift noise
            omega_noisy = omega + random.gauss(0.0, sqrt(sigma_omega_tr_2))
            omega_noisy += random.gauss(0.0, sqrt(sigma_omega_drift_2))


            # Move robot in the world (with noise):
            d = v_noisy * T
            dOmega = omega_noisy * T



            x = pose[0]
            y = pose[1]
            theta = pose[2]
            dx = x + d*cos(theta+0.5*dOmega)
            dy = y + d*sin(theta+0.5*dOmega)
            dTheta = theta + dOmega
            dpose = [dx, dy, dTheta]
            poseList.append(dpose)
        self.Particles = poseList
        return


    # 3c: Gewichtet alle Partikel mit dem Likelihoodfield-Algorithmus und führt ein Resampling durch.
    #     dist_list, alpha_list sind vom Roboter aufgenommene Laserdaten in Polarkoordinaten.
    def integrateMeasurement (self, dist_list, alpha_list, distantMap):
        pass


    # 3d: Berechnet aus der Partikelmenge eine Durchschnittspose.
    def getPose(self):
        pass


    # 3e: Berechnet die Kovarianz der Partikelmenge
    def getCovariance(self):
        pass


