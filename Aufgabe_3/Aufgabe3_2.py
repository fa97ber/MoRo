import numpy as np
from Robot_Simulator_V2 import simpleWorld


def generateDistanceGrid(world):
    myGrid = world.getDistanceGrid()
    #myGrid.drawGrid()
    return myGrid


if __name__ == "__main__":
    myWorld = simpleWorld.buildWorld()
    grid = generateDistanceGrid(myWorld)
    grid.drawGrid()