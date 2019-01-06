from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import Robot
import numpy as np
from Aufgabe_2.Kinematik import curveDrive, straightDrive


def wander(robot, v):

    while True:
        dist = robot.sense()
        n = -14
        possible = []
        for d in dist:
            if d is None:
                possible.append(n * 10)
            n += 1
            if n == 0:
                n += 1
        if len(possible) == len(dist):
            straightDrive(robot, v, 1.5, 50)
        elif possible.__contains__(-10) and possible.__contains__(10):
            straightDrive(robot, v, 1.5, 50)
        else:
            ran = np.random.randint(len(possible), size=1)
            curveDrive(robot, v, 0.01, possible[ran[0]], 50)
            #print(possible)

    return


if __name__ == "__main__":
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()

    #pln = [[3, 3], [3, 7], [7, 7], [7, 3]]
    #myWorld.drawPolyline(pln)
    myWorld.addLine(4, 9, 4, 13)
    myWorld.addLine(4, 13, 8, 13)
    myWorld.addLine(8, 13, 8, 9)

    myWorld.setRobot(myRobot, [6, 4, np.pi / 2])
    wander(myRobot, 1)


    myWorld.close()