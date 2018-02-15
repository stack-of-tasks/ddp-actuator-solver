from IPython import embed
from scipy.signal import hilbert
from matplotlib.pyplot import *
import numpy as np
from numpy import random
from numpy.random import rand
import csv


def frange(start, end=None, inc=None):
    "A range function, that does accept float increments..."

    if end == None:
        end = start + 0.0
        start = 0.0

    if inc == None:
        inc = 1.0

    L = []
    while 1:
        next = start + len(L) * inc
        if inc > 0 and next >= end:
            break
        elif inc < 0 and next <= end:
            break
        L.append(next)

    return L

nameList = []
list = []

''' position '''
path1 = './results.csv'

with open(path1,'r') as dataFile1:
    reader = csv.reader(dataFile1)
    i = 0
    for row in reader:
        if i == 2:
            for k in range(n+1):
                list[k].append(float(row[k]))
        if i == 1:
            i = 2;
            N = int(row[0])
            n = int(row[1])
            for k in range(n+1):
                list.append([])
        if i == 0:
            i = 1
            for k in row:
                nameList.append(k)

env1 = []
env2 = []
freq= []
for i in range(N):
    if list[5][i]==1:
        env1.append(20*np.log(list[2][i]))
        env2.append(20*np.log(list[3][i]))
        freq.append(list[5][i])
    if list[5][i]==-1:
        env1.append(20*np.log(-list[2][i]))
        env2.append(20*np.log(-list[3][i]))
        freq.append(list[5][i])

figure()
hold(1)
plot(list[6],list[1])
plot(list[6],list[2])
plot(list[6],list[3])
plot(list[6],list[4])
plot([list[6][0],list[5][-1]],[0.707,0.707],color='k',linewidth=2.0)
plot([list[6][0],list[5][-1]],[-0.707,-0.707],color='k',linewidth=2.0)

figure()
hold(1)
semilogx(freq,env1)
semilogx(freq,env2)

show()