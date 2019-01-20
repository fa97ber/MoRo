from Aufgabe_3 import ParticleFilterPoseEstimator
from Aufgabe_3 import Aufgabe3_1
from Aufgabe_3 import Aufgabe3_2
import numpy as np
from Robot_Simulator_V2 import simpleWorld
from Robot_Simulator_V2 import Robot
from PoseEstimator import PlotUtilities

if __name__ == "__main__":

    myWorld = simpleWorld.buildWorld()
    grid = Aufgabe3_2.generateDistanceGrid(myWorld)
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [4, 4, np.pi / 2])

    par = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator()
    par.initialize([3, 3, np.pi/3], [5, 5, (np.pi/3)*2], 200)

    # Roboterbewegung durchführen und anschließend als Referenz plotten
    n = 100
    v = 1
    r = 4
    theta = -np.pi

    poses = Aufgabe3_1.MCcurveDrive(myRobot, v, r, theta, n)

    # Messdaten, die für ein Resampling benötigt werden
    dist_list = myRobot.sense()
    alpha_list = myRobot.getSensorDirections()


    PlotUtilities.plotPositions(poses)

    PlotUtilities.plotPoseParticles(par.getParticles())   # Startpartikel generieren

    # Bewegung des Roboters nachsimulieren
    par.setRobot(myRobot)
    omega = v / r * np.sign(theta)
    for _ in range(n):
        par.integrateMovement([v/n, omega/n])
    PlotUtilities.plotPoseParticles(par.getParticles(), color='r') # End partikel generieren

    #for _ in range(20):
        #par.integrateMovement([1, -np.pi/8])
    #PlotUtilities.plotPoseParticles(par.getParticles(), color='g')
    par.integrateMeasurement(dist_list, alpha_list, grid)
    PlotUtilities.plotShow()


    #myWorld.close()
