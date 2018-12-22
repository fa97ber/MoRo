import numpy as np


def rot(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])


def rotx(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def roty(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotz(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot2trans(r):
    (n,m) = r.shape
    zh = np.zeros((1,n)) # horizontaler Nullvektor
    zv = np.zeros((n,1)) # vertikaler Nullvektor
    t = np.hstack((r,zv))
    zh = np.hstack((zh, [[1]]))
    return np.vstack((t,zh))




def trans(t):
    (n,m) = t.shape
    i = np.identity(n)
    z = np.zeros((1,n))
    t = np.hstack((i, t))
    z = np.hstack((z, [[1]]))
    return np.vstack((t, z))
