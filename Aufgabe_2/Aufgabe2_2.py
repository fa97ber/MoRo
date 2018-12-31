from Robot_Simulator_V2.graphics import *
from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import Robot
from Aufgabe_2.Kinematik import curveDrive, straightDrive
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


    # Orientierung der Geraden un des Roboters zueinander
    refv = np.array([[1], [0]])
    test = np.array([[-1], [0]])
    phigerade = np.arccos(scalar(n0v, refv) / (vectorLength(n0v) * vectorLength(refv)))
    if n0v[0] > 0 and n0v[1] > 0:
        phigerade = phigerade
    elif n0v[0] > 0 and n0v[1] < 0:
        phigerade = -phigerade
    #elif n0v[0] < 0 and n0v[1] < 0:
    #   phigerade = -phigerade - 90 / 180 * np.pi
    #elif n0v[0] < 0 and n0v[1] > 0:
    #    phigerade += 90 / 180 * np.pi

    print(dist, d)
    print(n0v)
    print(phigerade / np.pi * 180, theta / np.pi * 180)


    #P-Regler
    k = 1
    omega = -k * dist

    curveDrive(robot, 0.001, 0.001, (theta + phigerade) / np.pi * 180, 50)
    #-------------------------------------------------------------------------------------------
    #TODO: Ab hier alles prüfen ersetzen, da folgende Zeilen womöglich
    #	   nichts mit den erwünschten Reglern zu tun haben
    #-------------------------------------------------------------------------------------------
    #
    # # Roboter zur Linie schauen und fahren lassen falls nötig
    # while abs(dist) > 0.01:
    #     phi = phil - theta
    #     if dist > 0:
    #         phi -= np.pi
    #     phi = phi % (np.pi*2)
    #     #print(dist)
    #     #print("theta:", theta / np.pi * 180, "phi", phi / np.pi * 180)
    #     #print("phil", phil / np.pi * 180)
    #     if phi != 0:
    #         curveDrive(robot, 0.001, 0.001, phi / np.pi * 180, 50)
    #
    #     if dist != 0:
    #         straightDrive(robot, 1, abs(dist), 50)
    #
    #     (x, y, theta) = world.getTrueRobotPose()
    #     pr = np.array([[x], [y]])
    #     dist = scalar(pr, n0v) - d

    return


if __name__ == "__main__":
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()

    ln = [[12, 9.5], [13.0, 10.5]]
    myWorld.drawPolyline(ln)
    myRobot.setNoise(0, 0, 0)
    myWorld.setRobot(myRobot, [10, 10, np.pi])

    followLine(myRobot, ln[0], ln[1])

    myWorld.close()