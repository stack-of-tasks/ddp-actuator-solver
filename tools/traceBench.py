from matplotlib.pyplot import *
import csv

tauList = []
tauDotList = []
qList = []
qDotList = []
uList = []
tau2List = []
tauDot2List = []
q2List = []
qDot2List = []
u2List = []

''' position '''
path1 = '../_build/cpp/resultsBench.csv'
path2 = '../_build/cpp/resultsBenchMPC.csv'

with open(path1,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    for row in reader:
        if i ==1:
            tauList.append(float(row[2]))
            tauDotList.append(float(row[3]))
            qList.append(float(row[0]))
            qDotList.append(float(row[1]))
            uList.append((float(row[4])))
        if i==0:
            i = 1

with open(path2,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    for row in reader:
        if i ==1:
            tau2List.append(float(row[2]))
            tauDot2List.append(float(row[3]))
            q2List.append(float(row[0]))
            qDot2List.append(float(row[1]))
            u2List.append((float(row[4])))
        if i==0:
            i = 1


figure()

subplot(221)
plot(qList)
plot(q2List)
grid()

subplot(222)
plot(qDotList)
plot(qDot2List)
grid()

subplot(223)
plot(tauList)
plot(tau2List)
grid()

subplot(224)
plot(tauDotList)
plot(tauDot2List)
grid()

figure()
plot(uList)
plot(u2List)
grid()


show()



