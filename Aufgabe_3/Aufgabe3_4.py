from Aufgabe_3 import ParticleFilterPoseEstimator
from Aufgabe_3 import Aufgabe3_1
from Aufgabe_3 import Aufgabe3_2
import numpy as np
from Robot_Simulator_V2 import simpleWorld
from Robot_Simulator_V2 import Robot
from PoseEstimator import PlotUtilities
import matplotlib as plt

if __name__ == "__main__":

    myWorld = simpleWorld.buildWorld()
    #myWorld.addDynObstacleLine(10.5,2,10.5,5)
    grid = Aufgabe3_2.generateDistanceGrid(myWorld)
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [4, 4, np.pi / 2])




    par = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator()
    par.initialize([3, 3, np.pi/3], [5, 5, (np.pi/3)*2], 200)

    # Roboterbewegung durchführen und anschließend als Referenz plotten
    n = 100
    v = 1
    r = 4
    theta = -np.pi/2

    poses = Aufgabe3_1.MCcurveDrive(myRobot, v, r, theta, n)
    PlotUtilities.plotPositions(poses)

    # Messdaten, die für ein Resampling benötigt werden
    dist_list = myRobot.sense()
    alpha_list = myRobot.getSensorDirections()

    PlotUtilities.plotPoseParticles(par.getParticles(), color='b')   # Startpartikel generieren

    # Bewegung des Roboters nachsimulieren
    par.setRobot(myRobot)
    omega = v / r * np.sign(theta)
    for _ in range(int(n/2)):
        par.integrateMovement([v/n, omega/n])
    #PlotUtilities.plotPoseParticles(par.getParticles(), color='y') # Zwischen partikel generieren

    for _ in range(int(n/2)):
        par.integrateMovement([v/n, omega/n])
    #PlotUtilities.plotPoseParticles(par.getParticles(), color='r') # End partikel generieren

    par.integrateMeasurement(dist_list, alpha_list, grid)  #TODO: Diese Messung verändert alles

    #PlotUtilities.plotPoseParticles(par.getParticles(), color='g')



    # Zweite Bewegung und Messung

    poses = Aufgabe3_1.MCcurveDrive(myRobot, v, r, theta, n)
    PlotUtilities.plotPositions(poses)
    dist_list = myRobot.sense()
    alpha_list = myRobot.getSensorDirections()

    for _ in range(int(n / 2)):
        par.integrateMovement([v / n, omega / n])
    for _ in range(int(n / 2)):
        par.integrateMovement([v / n, omega / n])
    #PlotUtilities.plotPoseParticles(par.getParticles(), color='y')  # End partikel generieren
    print("Vorher", par.getPose())
    par.integrateMeasurement(dist_list, alpha_list, grid)

    print("Nachher", par.getPose())
    print("Original", myWorld.getTrueRobotPose())
    PlotUtilities.plotPoseParticles(par.getParticles(), color='b')


    plt.pyplot.plot([2,16,16,4,4,2,2], [2,2,12,12,10,10,2], 'k-')
    PlotUtilities.plotShow()


    myWorld.close()
