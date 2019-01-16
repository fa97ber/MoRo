import numpy as np
from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import Robot


def curveDrive(Robot, v, r, deltaTheta, n = 150):
    omega = v/r * np.sign(deltaTheta)
    deltaTheta = abs(deltaTheta / 180 * np.pi)
    # Anzahl Zeitschritte n mit jeweils der Laenge T = 0.1 sec definieren.
    # T laesst sich ueber die Methode myRobot.setTimeStep(T) einstellen.
    # T = 0.1 sec ist voreingestellt.
    Robot.setTimeStep(deltaTheta  / abs(omega))
    # Definiere Folge von Bewegungsbefehle:
    motionCircle = np.zeros((n, 2))
    for i in range(n):
        motionCircle[i] = [v/n, omega/n]
    # Bewege Roboter
    for t in range(n):
        # Bewege Roboter
        motion = motionCircle[t]
        #print("v = ", motion[0], "omega = ", motion[1]*180/np.pi)
        Robot.move(motion)
    return


def curveDriveAlt(robot, v, r, deltaTheta, n=100):
    omega = v/r * np.sign(deltaTheta)
    robot.setTimeStep(abs(deltaTheta / omega))

    # Bewege Roboter
    for t in range(n):
        robot.move([v/n, omega/n])
    return


def straightDrive(Robot, v, l, n = 150):
    Robot.setTimeStep(l/v)
    # Definiere Folge von Bewegungsbefehle:
    motionCircle = np.zeros((n,2))
    for i in range(n):
        motionCircle[i] = [v/n, 0]
    for t in range(n):
        # Bewege Roboter
        motion = motionCircle[t]
        #print("v = ", motion[0], "omega = ", motion[1]*180/np.pi)
        Robot.move(motion)
    return


if __name__ == "__main__":
    # Roboter in einer Welt positionieren:
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [2, 5.5, np.pi / 2])
    #myRobot.setNoise(0,0,0)
    curveDrive(myRobot, 1, 1, -90)
    straightDrive(myRobot, 1, 5)
    # Simulation schliessen:
    myWorld.close()

