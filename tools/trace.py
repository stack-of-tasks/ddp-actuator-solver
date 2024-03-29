#!/usr/bin/env python
import csv

import matplotlib.pyplot as pl

tauList = []
tauDotList = []
qList = []
qDotList = []
uList = []
""" position """
path = "../_build/cpp/results.csv"

with open(path, "r") as dataFile:
    reader = csv.reader(dataFile)
    i = 0
    for row in reader:
        if i == 1:
            tauList.append(float(row[0]))
            tauDotList.append(float(row[1]))
            qList.append(float(row[2]))
            qDotList.append(float(row[3]))
            uList.append((float(row[4])))
        if i == 0:
            i = 1

fig1 = pl.figure()
fig2 = pl.figure()

ax1 = fig1.add_subplot(221)
ax1.plot(tauList)
ax1.grid()

bx1 = fig1.add_subplot(222)
bx1.plot(tauDotList)
bx1.grid()

cx1 = fig1.add_subplot(223)
cx1.plot(qList)
cx1.grid()

dx1 = fig1.add_subplot(224)
dx1.plot(qDotList)
dx1.grid()

ax2 = fig2.add_subplot(111)
ax2.plot(uList)
ax2.grid()

pl.show()
