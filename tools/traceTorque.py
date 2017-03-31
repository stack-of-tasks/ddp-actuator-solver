from matplotlib.pyplot import *
import csv


tauList = []
tauDotList = []
qList = []
qDotList = []
uList = []

finaltauList = []
finaltauDotList = []
finalqList = []
finalqDotList = []
finaluList = []

tauListList = []
tauDotListList = []
qListList = []
qDotListList = []
uListList = []

''' position '''
path = '../_build/cpp/resultsBench.csv'

		
			
with open(path,'r') as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    j = -1
    for row in reader:
        if i ==1:
            tauList.append(float(row[0]))
            tauDotList.append(float(row[1]))
            qList.append(float(row[2]))
            qDotList.append(float(row[3]))
            uList.append((float(row[4])))
            uList.append((float(row[4])))
            j+=1
        if i==0:
            i = 1

tauList = tauList[100:]
qList = qList[100:]

joint_offset = tauList[0]
motor_offset = qList[0]

joint = [x-joint_offset for x in tauList]
motor = [x-motor_offset for x in qList]

print len(joint)
print len(motor)

time = []
torque = []
for i in range(len(joint)):
    time.append(i*0.001)
    torque.append(((motor[i]/96.1)-joint[i]))

figure()
plot(time,joint)
figure()
plot(time,motor)
figure()
plot(time,torque)

show()