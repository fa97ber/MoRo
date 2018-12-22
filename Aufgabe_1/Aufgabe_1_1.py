from Aufgabe_1.Transformation import *

#2.1a

#T A -> B

t = np.array([[-2],[0],[0]])
tm = trans(t)
print(tm)

r = rotz(np.pi)
rm = rot2trans(r)
print(rm)

print(np.dot(tm,rm))
