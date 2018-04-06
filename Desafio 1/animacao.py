#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
################################################

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


######################################################################
# Functions
def animate(i,data,pos,clear):
  if clear:
    # show i-th elements from data
    data.set_xdata(pos[i,0])
    data.set_ydata(pos[i,1])
  else:
    #show 0 to i-th elements from data
    data.set_xdata(pos[:i,0])
    data.set_ydata(pos[:i,1])



#######################################################################
# MAIN MODULE  
if __name__ == '__main__':
  
  fig = plt.figure()
  plt.ion() #interative mode
  plt.grid(True)
  
  ax = fig.add_subplot(111)  
  ax.set_xlim(-10, 10)
  ax.set_ylim(-20, 20) 
  
  data, = ax.plot([],[], color='blue', marker='o', markeredgecolor='r')
  
  # exemplo de array
  pose = np.array([[  0. , 0.        ],
 [  0.11858541  , 0.35575624],
 [  0.23717082 ,  0.71151247],
 [  0.35575624  , 1.06726871],
 [  0.47434165 , 1.42302495],
 [  0.59292706  , 1.77878118],
 [  0.71151247 ,  2.13453742],
 [  0.83009789 ,  2.49029366],
 [  0.9486833   , 2.84604989],
 [  1.06726871 ,  3.20180613],
 [  1.18585412 ,  3.55756237],
 [  1.30443953 ,  3.9133186 ],
 [  1.42302495 ,  4.26907484],
 [  1.54161036 ,  4.62483108],
 [  1.66019577 ,  4.98058731]])
  
  Ts = 100 #ms
  #função para fazer animação
  ani = FuncAnimation(fig, animate, np.arange(1, len(pose)), fargs=(data,pose,True),interval=Ts, repeat=False) 
  
  #show now
  plt.draw()
  plt.show()
  
  last_show = len(pose)
  
  try: 
    while True: #sai quando acontecer um erro (ex.: enviar sem vírgula o dado)
      #obtendo novas coordenadas
      dado= input("digite novo alvo - x,y: ")
      dado = dado.split(",")
      lista = [float(i) for i in dado]
      novo_alvo = np.array([lista])      
      print("x = {} e y = {}".format(novo_alvo[0,0],novo_alvo[0,1])) 
      print(novo_alvo)
      
      #adicionando ao vetor
      pose = np.append(pose, novo_alvo, axis=0)
  except:
    print("simulando novamente")
    pass 
  
  
  #simulando novamente com novo ponto
  ani = FuncAnimation(fig, animate, np.arange(last_show, len(pose)), fargs=(data,pose,False),interval=1000, repeat=False) 
  #show now
  plt.draw()
  plt.show()
  
  #espera para fechar
  input()