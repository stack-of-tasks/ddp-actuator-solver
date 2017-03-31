from matplotlib.pyplot import *
from numpy import random
from numpy.random import rand
import csv


tau1List = []
tauDot1List = []
q1List = []
qDot1List = []
u1List = []
tau2List = []
tauDot2List = []
q2List = []
qDot2List = []
u2List = []

tau2ListList = []
tauDot2ListList = []
q2ListList = []
qDot2ListList = []
u2ListList = []

klist = []

''' position '''
path1 = '../_build/cpp/resultsDC1.csv'
path2 = '../_build/cpp/resultsDC2.csv'

with open(path1,'r') as dataFile1:
    reader = csv.reader(dataFile1)
    i = 0
    j = 0
    for row in reader:
        if i ==1:
            tau1List.append(float(row[0]))
            tauDot1List.append(float(row[1]))
            q1List.append(float(row[2]))
            u1List.append((float(row[3])))
            klist.append((float(row[4])))
        if i==0:
            i = 1

with open(path2,'r') as dataFile2:
    reader = csv.reader(dataFile2)
    i = 0
    j = -1
    for row in reader:
        if i ==2:
            if (j < T):
                tau2List.append(float(row[0]))
                tauDot2List.append(float(row[1]))
                q2List.append(float(row[2]))
                u2List.append((float(row[3])))
                j += 1
            else:
                tau2ListList.append(tau2List)
                tauDot2ListList.append(tauDot2List)
                q2ListList.append(q2List)
                u2ListList.append(u2List)
                tau2List = [float(row[0])]
                tauDot2List = [float(row[1])]
                q2List = [float(row[2])]
                u2List = [float(row[3])]
                j = 0
        if i ==1:
            T = int(row[0])
            N = int(row[1])+1
            i = 2
        if i==0:
            i = 1

N = len(tau1List)/2
pos_openLopp_NoNoise = tau1List[0:N]
pos_closeLopp_NoNoise = tau1List[N:len(tau1List)]
pos_openLopp_Noise = tau2ListList[0]
pos_closeLopp_Noise = tau2ListList[1]
u_openLopp_NoNoise = u1List[0:N]
u_closeLopp_NoNoise = u1List[N:len(tau1List)]
u_openLopp_Noise = tau2ListList[0]
u_closeLopp_Noise = tau2ListList[1]

fichier = open('posSimu.csv','w')
fichier.write('time,pos_openLopp_NoNoise,pos_closeLopp_NoNoise,pos_openLopp_Noise,pos_closeLopp_Noise,u_openLopp_NoNoise,u_closeLopp_NoNoise,u_openLopp_Noise,u_closeLopp_Noise')
for i in range(N):
    fichier.write(str(float(0.001*i))+","+str(pos_openLopp_NoNoise[i])+","+str(pos_closeLopp_NoNoise[i])+","+
                  str(pos_openLopp_Noise[i])+","+str(pos_closeLopp_Noise[i])+","+str(u_openLopp_NoNoise[i])+
                  ","+str(u_closeLopp_NoNoise[i])+","+str(u_openLopp_Noise[i])+","+str(u_closeLopp_Noise[i])+"\n")
fichier.close()