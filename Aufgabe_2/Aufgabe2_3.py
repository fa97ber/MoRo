from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import officeWorld
from Robot_Simulator_V2 import Robot
import numpy as np
from Aufgabe_2.Kinematik import curveDrive, straightDrive

# Länge(Betrag) eines 2D-Vektors
def vectorLength(v):
    return np.sqrt(v[0]**2 + v[1]**2)


def wander(robot, v):

    while True:
        # alle Laser prüfen und mögliche Fahrtrichtungen speichern
        dist = robot.sense()
        n = -14
        possible = []
        for d in dist:
            if d is None or d > v * 1.5:
                possible.append(n * 10)
            n += 1
            if n == 0:
                n += 1

        # Fahrverhalten an die möglichen Fahrtrichtungen anpassen
        if robot.senseBoxes() is not None:
            tol = 1
            gotoToNextBox(robot, v, tol)
        elif len(possible) == len(dist):
            straightDrive(robot, v, v, v * 10)
        elif possible.__contains__(-10) and possible.__contains__(10) and possible.__contains__(-20) and possible.__contains__(20):
            straightDrive(robot, v, v, v * 10)
        elif len(possible) > 0:
            ran = np.random.randint(len(possible), size=1)
            #print(possible)
            curveDrive(robot, v, v / 100, possible[ran[0]], v * 10)

        else:
            curveDrive(robot, v, 0.0001, 180, v * 10)

    return


def gotoToNextBox(robot, v, tol):
    boxes = robot.senseBoxes()
    closeBox = None
    print("boxes: ", boxes)
    #print("close1:", closeBox)
    for x in range(0, len(boxes)):
        if len(boxes[x]) == 1:
            closeBox = boxes
        elif closeBox is None:
            closeBox = boxes[x]
        elif vectorLength(closeBox) > vectorLength(boxes[x]):
            closeBox = boxes[x]

    #print("close2:", closeBox)

    p1, p2 = closeBox
    print(p1, p2, closeBox)
    #p1 = p1[0]
    #p2 = p2[0]
    x = 0
    y = 0
    theta = 90
    k = 0.02
    # while vectorLength([p1, p2]) > tol:
    #     boxes = robot.senseBoxes()
    #     closeBox = None
    #     for x in range(0, len(boxes)):
    #         if len(boxes[x]) == 1:
    #             closeBox = boxes
    #         elif closeBox is None:
    #             closeBox = boxes[x]
    #         elif vectorLength(closeBox) > vectorLength(boxes[x]):
    #             closeBox = boxes[x]
    #     p1, p2 = closeBox
    #     p1 = p1[0]
    #     p2 = p2[0]
    thetaStern = np.arctan2(p2, p1) / np.pi * 180
    diff = (thetaStern - theta) % 360
    if diff > 180:
        diff = diff - 360
    print(diff)
    omega = k * diff
    robot.move([v, omega])
    return

if __name__ == "__main__":
    myWorld = emptyWorld.buildWorld()
    #myWorld = officeWorld.buildWorld()
    myRobot = Robot.Robot()

    # test1
    #myWorld.addLine(4, 9, 4, 13)
    #myWorld.addLine(4, 13, 8, 13)
    #myWorld.addLine(8, 13, 8, 9)

    # test2
    #myWorld.addLine(5, 4, 5, 13)
    #myWorld.addLine(5, 13, 7, 13)
    #myWorld.addLine(7, 13, 7, 4)
    myWorld.addBox(1, 1)
    myWorld.addBox(10, 10)


    myWorld.setRobot(myRobot, [6, 12, np.pi / 2])
    wander(myRobot, 1)


    myWorld.close()