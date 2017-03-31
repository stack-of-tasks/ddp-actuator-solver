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
desList = []

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
            desList.append((float(row[5])))
        if i==0:
            i = 1

tau2List = []
tauDot2List = []
q2List = []
qDot2List = []
u2List = []

tauListList = []
tauDotListList = []
qListList = []
qDotListList = []
uListList = []
with open(path2,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    j = -1
    for row in reader:
        if i ==2:
            if(j<=T):
                tau2List.append(float(row[2]))
                tauDot2List.append(float(row[3]))
                q2List.append(float(row[0]))
                qDot2List.append(float(row[1]))
                u2List.append((float(row[4])))
                j+=1
            else:
                tauListList.append(tau2List)
                tauDotListList.append(tauDot2List)
                qListList.append(q2List)
                qDotListList.append(qDot2List)
                uListList.append(u2List)
                tau2List = [float(row[2])]
                tauDot2List = [float(row[3])]
                q2List = [float(row[0])]
                qDot2List = [float(row[1])]
                u2List = [float(row[4])]
                j = 0
        if i==1:
            T = int(row[0])
            N = int(row[1]) - 1
            i = 2
        if i==0:
            i = 1
finaltauList = []
finaltauDotList = []
finalqList = []
finalqDotList = []
finaluList = []
for i in range(N):
    finaltauList.append(tauListList[i][0])
    finaltauDotList.append(tauDotListList[i][0])
    finalqList.append(qListList[i][0])
    finalqDotList.append(qDotListList[i][0])
    finaluList.append(uListList[i][0])

figure()
plot(qList[0:len(qList)-1])
plot(desList[0:len(qList)-1])
grid()

figure()
plot(finaluList)
grid()

'''figure()

subplot(221)
plot(qList)
plot(desList)
for i in range(0,N,100):
    plot(range(i,i+T+1,1),qListList[i][:len(qListList[i])-1])
grid()

subplot(222)
plot(qDotList)
for i in range(0,N,100):
    plot(range(i,i+T+1,1),qDotListList[i][:len(qListList[i])-1])
grid()

subplot(223)
plot(tauList)
for i in range(0,N,100):
    plot(range(i,i+T+1,1),tauListList[i][:len(qListList[i])-1])
grid()

subplot(224)
plot(tauDotList)
for i in range(0,N,100):
    plot(range(i,i+T+1,1),tauDotListList[i][:len(qListList[i])-1])
grid()

figure()
plot(uList)
plot(finaluList,'x')
for i in range(0,N,100):
    plot(range(i,i+T+1,1),uListList[i][:len(qListList[i])-1])
grid()'''


show()



