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

        T = self.robot.getTimeStep()

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

    def __ndf(self,x,mu,sigma2):
        c = 1/(np.sqrt(2*sigma2*np.pi))
        return c*np.exp(-0.5 * (mu-x)**2/sigma2)

    # 3c: Gewichtet alle Partikel mit dem Likelihoodfield-Algorithmus und führt ein Resampling durch.
    #     dist_list, alpha_list sind vom Roboter aufgenommene Laserdaten in Polarkoordinaten.
    def integrateMeasurement (self, dist_list, alpha_list, distantMap):
        n = 0
        obstacles = []
        minDistance = np.inf
        for dist in dist_list:
            if dist is not None:
                distance = dist
                angle = alpha_list[n]
                obstacles.append([distance, angle])
                if distance < minDistance:
                    minDistance = distance
            n += 1

        #print(obstacles)
        chance = []
        weightedParticles = []    # Diese Liste enthält die Partikel mehrfach entsprechend ihrem Gewicht
        #weightedParticles.append([0.0, 0.0, None])
        ##tol = 0.2
        #weight = 10                # Zusätzliches Gewicht, das passende Partikel bekommen
        for m in range(len(self.Particles)):
            pose = self.Particles[m]
            pdist = distantMap.getValue(pose[0], pose[1]) # Entfernungswert zum nächsten Hindernis aus dem Likelihoodfield

            if pdist is not None:
                prob = 1.0
                for obstacle in obstacles:
                    x = pose[0] + obstacle[0] * np.cos(obstacle[1] + pose[2])
                    y = pose[1] + obstacle[0] * np.sin(obstacle[1] + pose[2])
                    dist = distantMap.getValue(x, y)
                    if dist is not None:
                        p = self.__ndf(0, dist, 0.4**2)
                        prob = prob * p
                #weightedParticles.append(pose)   # Jedes Partikel, das einen Entfernungswert über die Map hat wird mit 1 gewichtet
                #if minDistance - tol <= pdist <= minDistance + tol:
                    #for _ in range(int(weight)):
                        #weightedParticles.append(pose) # Jedes Partikel, dessen Wert innerhalb der Toleranz liegt
                                                       # wird zuätzlich in die Liste gespeichert und damit höher gewichtet
                #lastweight = weightedParticles[-1][1]
                #weightedParticles.append([lastweight, lastweight + prob, m])
                chance.append(prob)
                weightedParticles.append(m)


        # Die Partikelliste wird neu aufgebaut
        chance = chance/np.sum(chance)
        poseList = []
        #ran = np.random.uniform(0, weightedParticles[-1][1], len(self.Particles))
        ran = np.random.choice(weightedParticles, size = len(self.Particles), p = chance)
        print(ran)
        print(chance)
        print(weightedParticles)
        #print(weightedParticles)
        for r in ran:
            #for n in range(len(weightedParticles)):
                #if weightedParticles[n][0] < r < weightedParticles[n][1]:
                    #index = weightedParticles[n][2]
                    #if index is not None:
            poseList.append(self.Particles[n])
                    #continue
        self.Particles = poseList
        return


    # 3d: Berechnet aus der Partikelmenge eine Durchschnittspose.
    def getPose(self):
        return np.mean(self.Particles, axis=0)


    # 3e: Berechnet die Kovarianz der Partikelmenge
    def getCovariance(self):
        return np.cov(self.Particles[:,:3].T)


