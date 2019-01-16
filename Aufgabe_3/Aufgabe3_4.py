from Aufgabe_3 import ParticleFilterPoseEstimator
from Aufgabe_3 import Aufgabe3_1
import numpy as np
from Robot_Simulator_V2 import simpleWorld
from Robot_Simulator_V2 import Robot
from PoseEstimator import PlotUtilities

if __name__ == "__main__":

    #myWorld = simpleWorld.buildWorld()
    #myRobot = Robot.Robot()
    #myWorld.setRobot(myRobot, [4, 4, np.pi / 2])
    par = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator()
    par.initialize([3, 3, np.pi/2], [6, 6, np.pi])
    #Aufgabe3_1.MCcurveDrive(myRobot)
    PlotUtilities.plotPoseParticles(par.getParticles())
    par.integrateMovement([1, -180])
    par.integrateMovement([1, -180])
    par.integrateMovement([1, -180])
    par.integrateMovement([1, -180])
    par.integrateMovement([1, -180])
    PlotUtilities.plotPoseParticles(par.getParticles(), color='r')
    PlotUtilities.plotShow()
    #myWorld.close()
