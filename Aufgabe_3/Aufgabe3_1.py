import numpy as np
from Robot_Simulator_V2 import simpleWorld
from Robot_Simulator_V2 import Robot


def MCcurveDrive(robot, v=1, r=4, deltaTheta=-np.pi, n=100):
    omega = v/r * np.sign(deltaTheta)
    robot.setTimeStep(abs(deltaTheta / omega))

    world = robot.getWorld()
    poses = []
    poses.append(world.getTrueRobotPose())
    # Bewege Roboter
    for t in range(n):
        robot.move([v/n, omega/n])
        poses.append(world.getTrueRobotPose())
    return poses


if __name__ == "__main__":
    # Roboter in einer Welt positionieren:
    myWorld = simpleWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [4, 4, np.pi / 2])
    #myRobot.setNoise(0,0,0)
    MCcurveDrive(myRobot)
    # Simulation schliessen:
    myWorld.close()