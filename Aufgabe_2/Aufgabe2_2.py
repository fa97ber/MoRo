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


def followLine(robot, p1, p2):
    (x1, y1) = p1
    (x2, y2) = p2

    #Vektoren von p1, p2 und der Linie l bestimmen
    p1v = np.array([[x1], [y1]])
    p2v = np.array([[x2], [y2]])
    lv = np.array([[x2-x1], [y2-y1]])

    #Normalenform der Geraden 
    #bestimmen : g(xv): (xv-pv) * nv = 0
    nv = np.array([[-(y2-y1)], [x2-x1]])
    pv = p1v

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
	
	#-------------------------------------------------------------------------------------------
	#TODO: Ab hier alles prüfen ersetzen, da folgende Zeilen womöglich 
	#	   nichts mit den erwünschten Reglern zu tun haben
	#-------------------------------------------------------------------------------------------
	
    refv = np.array([[1], [0]])
    phil = np.arccos(scalar(n0v, refv) / (vectorLength(n0v) * vectorLength(refv)))
    # Roboter zur Linie schauen und fahren lassen falls nötig
    while abs(dist) > 0.01:
        phi = phil - theta
        if dist > 0:
            phi -= np.pi
        phi = phi % (np.pi*2)
        #print(dist)
        #print("theta:", theta / np.pi * 180, "phi", phi / np.pi * 180)
        #print("phil", phil / np.pi * 180)
        if phi != 0:
            curveDrive(robot, 0.001, 0.001, phi / np.pi * 180, 50)

        if dist != 0:
            straightDrive(robot, 1, abs(dist), 50)

        (x, y, theta) = world.getTrueRobotPose()
        pr = np.array([[x], [y]])
        dist = scalar(pr, n0v) - d

    return


if __name__ == "__main__":
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()

    ln = [[12, 10.5], [12.0, 15.5]]
    myWorld.drawPolyline(ln)
    #myRobot.setNoise(0, 0, 0)
    myWorld.setRobot(myRobot, [2, 5.5, np.pi])

    followLine(myRobot, ln[0], ln[1])

    myWorld.close()