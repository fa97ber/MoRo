
import numpy as np
from Robot_Simulator_V2 import simpleWorld
from Robot_Simulator_V2 import Robot


def curveDrive(robot, v, r, deltaTheta, n=100):
    omega = v/r * np.sign(deltaTheta)
    robot.setTimeStep(abs(deltaTheta / omega))

    # Bewege Roboter
    for t in range(n):
        robot.move([v/n, omega/n])
    return


if __name__ == "__main__":
    # Roboter in einer Welt positionieren:
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [4, 4, np.pi / 2])
    #myRobot.setNoise(0,0,0)
    curveDrive(myRobot, 1, 4, -np.pi)
    # Simulation schliessen:
    myWorld.close()