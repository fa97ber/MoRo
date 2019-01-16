
from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import Robot

import numpy as np


# Skalarprodukt von zwei 2D-Vektoren
def scalar(v1, v2):
    return (v1[0] * v2[0] + v1[1] * v2[1])[0]


# Länge(Betrag) eines 2D-Vektors
def vectorLength(v):
    return (np.sqrt(v[0]**2 + v[1]**2))[0]


def followLine(robot, p, q):
    p1, p2 = p
    q1, q2 = q

    #Vektoren von p, q und der Linie l bestimmen
    pv = np.array([[p1], [p2]])
    qv = np.array([[q1], [q2]])
    lv = np.array([[q1-p1], [q2-p2]])

    #Normalenform der Geraden
    #bestimmen : g(xv): (xv-pv) * nv = 0
    nv = np.array([[-(q2-p2)], [q1-p1]])
    #pv = pv

    #Hessesche Normalform der Gerade
    #berechnen: g(xv): xv * n0v = d
    if scalar(pv, nv) >= 0:
        n0v = nv/vectorLength(nv)
    else:
        n0v = -(nv/vectorLength(nv))
    d = scalar(pv, n0v)

    #Robotposition bestimmen:
    world = robot.getWorld()
    (x, y, theta) = world.getTrueRobotPose()
    pr = np.array([[x], [y]])


    #Abstand und Ausrichtung des Roboters zur Linie bestimmen
    dist = scalar(pr, n0v) - d
    distalt = dist
    #print(dist)

    # Orientierung der Geraden im KS bestimmen
    refv = np.array([[1], [0]])
    phigerade = np.arccos(scalar(n0v, refv) / (vectorLength(n0v) * vectorLength(refv)))
    if (n0v[0] > 0 and n0v[1] < 0) or (n0v[0] < 0 and n0v[1] < 0):
        phigerade = -phigerade

    #print(dist, d)
    #print(n0v)
    #print(phigerade / np.pi * 180, theta / np.pi * 180)

    #Regler
    kp = 0.4  # Modifikator der Winkelgeschwindigkeit: P-Anteil
    kd = 1

    if phigerade - theta < 0:
        kp = -kp
    #omega = -kp * dist - kd * ((dist - distalt) / robot.getTimeStep())
    #print(omega / np.pi * 180)
    v = 1    # Geschwindigkeit
    while True:
        (x, y, theta) = world.getTrueRobotPose()
        pr = np.array([[x], [y]])

        dist = scalar(pr, n0v) - d
        omega = -kp * dist - kd * ((dist - distalt) / robot.getTimeStep())
        #print("dist", dist, "omega", omega / np.pi * 180)


        if not robot.move([abs(v), omega]):
            break;
        distalt = dist

    return


def gotoGlobal(robot, v, p, tol):
    p1, p2 = p

    # Robotposition bestimmen:
    world = robot.getWorld()
    (x, y, theta) = world.getTrueRobotPose()

    k = 1
    while vectorLength(np.array([[p1-x], [p2-y]])) > tol:
        (x, y, theta) = world.getTrueRobotPose()
        theta = theta / np.pi * 180
        thetaStern = np.arctan2(p2 - y, p1 - x) / np.pi * 180
        diff = (thetaStern - theta) % 360
        #print(thetaStern, theta, diff)
        if diff > 180:
            diff = diff - 360
        #print(diff)
        omega = k * (diff / 180 * np.pi)
        robot.move([v, omega])
    return


def followPolyline(robot, v, line):

    tol = 1
    for point in line:
        if point == line[-1]:
            tol = 0.1
        gotoGlobal(robot, v, point, tol)


if __name__ == "__main__":
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()

    ln = [[4, 10], [13.0, 10]]
    pln = [[2, 7], [4, 10], [13.0, 10], [13, 2]]
    myWorld.drawPolyline(ln)
    #myRobot.setNoise(0, 0, 0)
    myWorld.setRobot(myRobot, [1, 9, 0])

    followLine(myRobot, ln[0], ln[1])

    #gotoGlobal(myRobot, 1, [13, 10], 0.1)
    #followPolyline(myRobot, 0.5, pln)
    myWorld.close()
