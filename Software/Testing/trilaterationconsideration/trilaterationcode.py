import numpy as np

#inputs
P_i_m = np.array([[10,10,5],[-10,10,0],[-10,-10,-5]])
r_m = np.array([20,20,20])
N = 3


#other vars
P_0_m = np.array([10,10,10])
I = np.array([1,0,0],
             [0,1,0],
             [0,0,1])


def calcA():
    sumtotal = 0
    for i in range(0,N):
        sumtotal = sumtotal + P_i_m[i]*np.transpose(P_i_m[i])*P_i_m[i] - pow(r_m,2)*P_i_m[i]
    return (1/N)*sumtotal

def calcB():
    sumtotal = 0
    for i in range(0,N):
        sumtotal = sumtotal + (-2*P_i_m[i]*np.transpose(P_i_m[i])) - ((np.transpose(P_i_m[i])*P_i_m[i])*I) + (pow(r_m,2)*I)
    return (1/N)*sumtotal

def calcC():
    sumtotal = 0
    for i in range(0,N):
        sumtotal = sumtotal + P_i_m
    return (1/N)*sumtotal

a = calcA()
B = calcB()
c = calcC()
D = B + 2*c*np.transpose(c) + ((c*np.transpose(c))*I)

f = a + B*c + 2*c*np.transpose(c)*c
#H = D-