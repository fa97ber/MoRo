import numpy as np


# 3a: Erzeugt n zufällige Partikel in dem vorgegebenen Posen-Bereich.
def initialize(self, poseFrom, poseTo, n=200):
    pass


# 3b: Wendet auf alle Partikel den Bewegungsbefehl motion mit einem zufälligen Rauschen an.
def integrateMovement(self, motion):
    pass


# 3c: Gewichtet alle Partikel mit dem Likelihoodfield-Algorithmus und führt ein Resampling durch.
#     dist_list, alpha_list sind vom Roboter aufgenommene Laserdaten in Polarkoordinaten.
def integrateMeasurement (self, dist_list, alpha_list, distantMap):
    pass


# 3d: Berechnet aus der Partikelmenge eine Durchschnittspose.
def getPose(self):
    pass


# 3e: Berechnet die Kovarianz der Partikelmenge
def getCovariance(self):
    pass


if __name__ == "__main__":
    pass