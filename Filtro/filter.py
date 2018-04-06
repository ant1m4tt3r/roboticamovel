#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
################################################

import math as m
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


######################################################################
# Functions

def apply_filter(vel, tau, Ts):
    vf = [0]
    for i in range(len(vel)):
        v = (Ts / (tau + Ts)) * vel[i] + (tau / (tau + Ts)) * vf[-1]
        vf.append(v)
    
    return vf
    

#######################################################################
# MAIN MODULE  
if __name__ == '__main__':
  
    file = open("encoder.txt",'r')
    linhas = file.readlines()
    t = [0]
    vel = []
    for linha in linhas:
        (t_i, vel_i) = linha.split("\t")
        t.append(float(t_i))
        vel.append(float(vel_i))
    
    file.close()

    ts = 0

    for i in range(len(t) -1):
        ts += t[i+1] - t[i]

    Ts = ts / len(t)
    tau = 4 * Ts

    vf = apply_filter(vel, tau, Ts)
    vel.insert(0, 0)

   
    fig = plt.figure()
   
    plt.ion() #interative mode
    plt.grid(True)

    ax = fig.add_subplot(111)    
    
    ax.plot(t,vel, color='red', marker='*', markeredgecolor='r')
    ax.plot(t,vf, color='blue', marker='*', markeredgecolor='b')
    ax.set_xlabel('t (s)')
    ax.set_ylabel('$\omega$ (rad/s)')

    plt.show()

    input()
  
  