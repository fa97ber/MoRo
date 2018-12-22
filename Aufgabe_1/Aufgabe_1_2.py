from Aufgabe_1.Transformation import *

#a)
#Roboter {R}
xR = 2
yR = 1
theta = np.pi / 6
r = 0.1
tlR = trans(np.array([[xR],[yR],[r]]))
rotR = rot2trans(rotz(theta))
transR = np.dot(tlR,rotR)
#print(tlR)
#print(rotR)
#print("R:")
#print(transR)

#Drehtellerbasis DB
l = 0.6
h = 0.2
a = 0.1
tlDB = trans(np.array([[l/2 - a/2],[0],[h]]))
rotDB = rot2trans(rotz(0))
transDB = np.dot(tlDB,rotDB)
#print(tlDB)
#print(rotDB)
#print("DB:")
#print(transDB)

#Drehteller D
b = 0.1
alpha = 2/9 * np.pi  #40°

tlD = trans(np.array([[0],[0],[b/2]]))
rotD1 = rot2trans(rotz(alpha))
rotD2 = rot2trans(rotx(np.pi/2))
transD = tlD.dot(rotD1).dot(rotD2)

#print(transD)

#Armteil 1 A1
l1 = 0.5
beta1 = 1/6 * np.pi
tlA1_1 = trans(np.array([[0],[0],[a/2]]))
rotA1 = rot2trans(rotz(beta1))
tlA1_2 = trans(np.array([[l1],[0],[0]]))
transA1 = tlA1_1.dot(rotA1).dot(tlA1_2)

#Armteil 2 A2
l2 = 0.5
beta2 = -1/18 * np.pi
rotA2 = rot2trans(rotz(beta2))
tlA2 = trans(np.array([[l2],[0],[0]]))
transA2 = rotA2.dot(tlA2)

#Punkt P

pA2 = np.array([[0],[0],[0],[1]])

pO = transR.dot(transDB).dot(transD).dot(transA1).dot(transA2).dot(pA2)
#pR1 = transDB.dot(transD).dot(transA1).dot(transA2).dot(pA2)
#print(pR1)
print("pO:")
print(pO)

#b)
print("\nAufgabe 2b:\n")


def vorwKb(v_alpha, v_beta1, v_beta2, p_A2):

    #Drehtellerbasis DB
    l = 0.6
    h = 0.2
    a = 0
    tlDB = trans(np.array([[l/2 - a/2],[0],[h]]))
    rotDB = rot2trans(rotz(0))
    transDB = np.dot(tlDB,rotDB)


    #Drehteller D
    b = 0
    alpha = v_alpha

    tlD = trans(np.array([[0],[0],[b/2]]))
    rotD1 = rot2trans(rotz(alpha))
    rotD2 = rot2trans(rotx(np.pi/2))
    transD = tlD.dot(rotD1).dot(rotD2)

    #Armteil 1 A1
    l1 = 0.5
    beta1 = v_beta1

    tlA1_1 = trans(np.array([[0],[0],[a/2]]))
    rotA1 = rot2trans(rotz(beta1))
    tlA1_2 = trans(np.array([[l1],[0],[0]]))
    transA1 = tlA1_1.dot(rotA1).dot(tlA1_2)

    #Armteil 2 A2
    l2 = 0.5
    beta2 = v_beta2
    rotA2 = rot2trans(rotz(beta2))
    tlA2 = trans(np.array([[l2],[0],[0]]))
    transA2 = rotA2.dot(tlA2)

    return transDB.dot(transD).dot(transA1).dot(transA2).dot(p_A2)

def invK(Orig):
    #Berechnung von alpha
    Z = np.copy(Orig)
    xz,yz,zz = Z[0][0],Z[1][0],Z[2][0]
    xz = xz -l/2
    zz = zz - h
    Z[0][0],Z[1][0],Z[2][0] = xz,yz,zz

    alpha = np.arctan2(yz, xz)
    #Rotieren von P um -alpha, um alpha aus der Rechnung zu Entfernen
    r = rotz(-alpha)
    r = rot2trans(r)
    Z = r.dot(Z)

    #Berechnung von beta1 und beta2
    xz,yz,zz = Z[0][0],Z[1][0],Z[2][0]


    a = np.sqrt(xz**2 + zz**2)
            #Berechnung für b und c:
            #l2**2 = c**2 + b**2
            #a**2 = b**2 + (l1+c)**2
            #
            #b = np.sqrt(l2**2-c**2)
            #a**2 = l2**2-c**2 + (l1**2 + 2*l1*c + c**2)
            #a**2 = l2**2 + l1**2 + 2*l1*c
            #a**2 - l2**2 - l1**2 = 2*l1*c
            #(a**2-l2**2)/(2*l1) - l1/2 = c


    c = (a**2-l2**2)/(2*l1) - l1/2
    b = -np.sqrt(l2**2-c**2)
    beta1_1 = np.arctan2(zz,xz)
    beta1_2 = np.arctan2(-b,l1+c)
    beta1 = beta1_1 + beta1_2
    beta2 = np.arctan2(b,c)
    return alpha,beta1,beta2


pR = vorwKb(40 * np.pi / 180, 30 * np.pi / 180, -10 * np.pi / 180, np.array([[0],[0],[0],[1]]))
print("VK P:\n", pR)

alpha_i, beta1_i, beta2_i = invK(pR)
print("alpha, beta1, beta2:\n", alpha_i * 180 / np.pi, beta1_i * 180 / np.pi, beta2_i * 180 / np.pi)




#c)
print("Aufgabe2c:\n")
rangeARR = np.linspace(0.1, 0.3, 3)
for x in rangeARR:
    for y in rangeARR:
        for z in rangeARR:
            pC = np.array([[x], [y], [z], [1]])
            alpha_c, beta1_c, beta2_c = invK(pC)
            print("Punkt:\n", pC)
            print("alpha, beta1, beta2:\n", alpha_c * 180 / np.pi, beta1_c * 180 / np.pi, beta2_c * 180 / np.pi)
            print("VK Punkt:\n", vorwKb(alpha_c, beta1_c, beta2_c, np.array([[0],[0],[0],[1]])))
