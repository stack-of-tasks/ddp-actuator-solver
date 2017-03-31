from matplotlib.pyplot import *
import csv


tauList1 = []
tauDotList1 = []
qList1 = []
qDotList1 = []
u1List1 = []
u2List1 = []
tauList2 = []
tauDotList2 = []
qList2 = []
qDotList2 = []
u1List2 = []
u2List2 = []

tau2ListList = []
tauDot2ListList = []
q2ListList = []
qDot2ListList = []
u1ListList = []
u2ListList = []

''' position '''
path1 = '../_build/cpp/results1.csv'
path2 = '../_build/cpp/results2.csv'

with open(path1,'r') as dataFile1:
    reader = csv.reader(dataFile1)
    i = 0
    j = 0
    for row in reader:
        if i ==1:
            tauList1.append(float(row[0]))
            tauDotList1.append(float(row[1]))
            qList1.append(float(row[2]))
            qDotList1.append(float(row[3]))
            u1List1.append((float(row[4])))
            u2List1.append((float(row[5])))
        if i==0:
            i = 1

with open(path2,'r') as dataFile2:
    reader = csv.reader(dataFile2)
    i = 0
    j = -1
    for row in reader:
        if i ==2:
            if (j < T):
                tauList2.append(float(row[0]))
                tauDotList2.append(float(row[1]))
                qList2.append(float(row[2]))
                qDotList2.append(float(row[3]))
                u1List2.append((float(row[4])))
                u2List2.append((float(row[5])))
                j += 1
            else:
                tau2ListList.append(tauList2)
                tauDot2ListList.append(tauDotList2)
                q2ListList.append(qList2)
                qDot2ListList.append(qDotList2)
                u1ListList.append(u1List2)
                u2ListList.append(u2List2)
                tauList2 = [float(row[0])]
                tauDotList2 = [float(row[1])]
                qList2 = [float(row[2])]
                qDotList2 = [float(row[3])]
                u1List2 = [float(row[4])]
                u2List2 = [float(row[5])]
                j = 0
        if i ==1:
            T = int(row[0])
            N = int(row[1]) - 1
            i = 2
        if i==0:
            i = 1

fig1 = figure()

subplot(221)
hold(1)
plot(tauList1)
for i in range(N):
    plot(tau2ListList[i])
title('joint position',fontsize=32)
grid()

subplot(222)
hold(1)
plot(tauDotList1)
for i in range(N):
    plot(tauDot2ListList[i])
title('joint speed',fontsize=32)
grid()

subplot(223)
hold(1)
plot(qList1)
for i in range(N):
    plot(q2ListList[i])
title('motor position',fontsize=32)
grid()

subplot(224)
hold(1)
plot(qDotList1)
for i in range(N):
    plot(qDot2ListList[i])
title('motor current',fontsize=32)
grid()


figure()
hold(1)
subplot(211)
plot(u1List1)
grid()
subplot(212)
plot(u2List1)
grid()


show()

