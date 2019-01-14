from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import officeWorld
from Robot_Simulator_V2 import Robot
import numpy as np
from Aufgabe_2.Kinematik import curveDrive, straightDrive
from Aufgabe_2.Aufgabe2_2 import scalar, vectorLength
from Robot_Simulator_V2 import SensorUtilities

# Länge(Betrag) eines 2D-Vektors
def lengthVector(v):
    return np.sqrt(v[0]**2 + v[1]**2)


def wander(robot, v):

    while True:
        # alle Laser prüfen und mögliche Fahrtrichtungen speichern
        dist = robot.sense()
        directions = robot.getSensorDirections()
        n = 0
        possible = []
        for d in dist:
            if d is None or d > v * 1.5:
                possible.append(directions[n] / np.pi * 180)
            n += 1
        # Fahrverhalten an die möglichen Fahrtrichtungen anpassen
        if len(SensorUtilities.extractLinesFromSensorData(dist, directions)) != 0:
            followWall(robot, v, 2)
        elif robot.senseBoxes() is not None and possible.__contains__(-5) and possible.__contains__(5) and possible.__contains__(-15) and possible.__contains__(15):
            tol = 1
            gotoToNextBox(robot, v, tol)

        elif len(possible) == len(dist):
            straightDrive(robot, v, v, 50)
        elif possible.__contains__(-5) and possible.__contains__(5) and possible.__contains__(-15) and possible.__contains__(15):
            straightDrive(robot, v, v, 50)
        elif len(possible) > 0:
            ran = np.random.randint(len(possible), size=1)
            curveDrive(robot, v, v / 100, possible[ran[0]], 50)

        else:
            curveDrive(robot, v, 0.0001, 180, 50)

    return


def gotoToNextBox(robot, v, tol):
    boxes = robot.senseBoxes()
    closeBox = None
    #print("boxes: ", boxes)
    for x in range(0, len(boxes)):
        print(boxes[x])
        if len(boxes[x]) == 1:
            closeBox = np.copy(boxes)
            closeBox[0] = closeBox[0][0]
            closeBox[1] = closeBox[1][0]
        elif closeBox is None:
            closeBox = boxes[x]
        elif closeBox[0] > boxes[x][0]:
            closeBox = boxes[x]

    dist, angl = closeBox
    #print(dist, angl, closeBox)

    k = 0.5
    omega = -k * angl
    robot.move([v, omega])
    return


def followWall(robot, v, d):
    dist = robot.sense()
    directions = robot.getSensorDirections()
    lines = SensorUtilities.extractLinesFromSensorData(dist, directions)
    print(lines)
    for line in lines:
        #print(line)
        p = line[0]
        q = line[1]
        #print("p, q", p, q)
        p1, p2 = p
        q1, q2 = q

        pv = np.array([[p1], [p2]])

        # Normalenform der Geraden
        # bestimmen : g(xv): (xv-pv) * nv = 0
        nv = np.array([[-(q2 - p2)], [q1 - p1]])

        # Hessesche Normalform der Gerade
        # berechnen: g(xv): xv * n0v = d
        if scalar(pv, nv) >= 0:
            n0v = nv / vectorLength(nv)
        else:
            n0v = -(nv / vectorLength(nv))
        dv = scalar(pv, n0v)

        # Roboterposition im eigenen KS
        (x, y, theta) = (0, 0, np.pi/2)
        pr = np.array([[x], [y]])

        # Abstand des Roboters zur Linie bestimmen
        dist = scalar(pr, n0v) - dv
        print(dist)

        # Orientierung der Geraden im KS bestimmen
        refv = np.array([[1], [0]])
        phigerade = np.arccos(scalar(n0v, refv) / (vectorLength(n0v) * vectorLength(refv)))
        if n0v[0] > 0 and n0v[1] > 0:
            phigerade = phigerade
        elif n0v[0] > 0 and n0v[1] < 0:
            phigerade = -phigerade
        print(phigerade / np.pi * 180)
        dist = abs(dist)

        if dist >= d-0.01 and dist <= d + 0.01:
            if phigerade > 0:
                curveDrive(robot, v, 0.001, (phigerade / np.pi * 180) - 90, 50) #Im rechten Winkel zur Gerade ausrichten
            else:
                curveDrive(robot, v, 0.001, (phigerade / np.pi * 180) + 90, 50)  # Im rechten Winkel zur Gerade ausrichten
            straightDrive(robot, v, v, 50)
        elif dist < d:
            curveDrive(robot, v, 0.001, phigerade / np.pi * 180 - 180, 50) #zur Gerade ausrichten
            straightDrive(robot, v, d - dist, 50)


            #TODO: von der Gerade weggdrehen
        elif dist > d:
            curveDrive(robot, v, 0.001, phigerade / np.pi * 180, 50)  # zur Gerade ausrichten
            straightDrive(robot, v, abs(d - dist), 50)
            # TODO: zu der Gerade hingdrehen
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
    myWorld.addLine(13, 13, 13, 4)
    #myWorld.addBox(1, 1)
    #myWorld.addBox(12, 10)


    myWorld.setRobot(myRobot, [10, 13, (np.pi / 2)* 3])
    wander(myRobot, 1)


    myWorld.close()
