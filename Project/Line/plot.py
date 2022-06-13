import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
from csv import writer
import numpy as np
import math


def plot():

  n=np.arange(1,26)



  actual = pd.read_csv("States.csv")
  target =pd.read_csv("target_states.csv")
  x1 = []
  y1 = []
  theta1=[]

  x2 = []
  y2 = []
  theta2=[]

  x1 = actual[0:61]['x']
  y1 = actual[0:61]['y']
  theta1 = actual[0:71]['theta']

  x2 = target[0:71]['x']
  y2 = target[0:71]['y']
  theta2 = target[0:71]['theta']

  MSE = np.square(np.subtract(x1,x2)).mean()+np.square(np.subtract(y1,y2)).mean()+np.square(np.subtract(theta1,theta2)).mean() 
  RMSE = math.sqrt(MSE)
  print("Root Mean Square Error:\n")
  print(RMSE)

  plt.figure(1)
  plt.plot(x1,y1,x2,y2) 
  location = 0
  legend_drawn_flag = True 
  plt.legend(["Actual", "Target"], loc=0, frameon=legend_drawn_flag)
  plt.suptitle("Tracking")
  plt.savefig('Tracking.png')

  plt.figure(2)
  plt.subplot(311)
  plt.plot(n,x1,n,x2)
  plt.ylabel('X')
  plt.subplot(312)
  plt.plot(n,y1,n,y2)
  plt.ylabel('Y')
  plt.subplot(313)
  plt.plot(n,theta1,n,theta2)
  plt.ylabel('Heading')

  location = 0
  legend_drawn_flag = True 
  plt.legend(["Actual", "Target"], loc=0, frameon=legend_drawn_flag)
  plt.suptitle("States")
  plt.savefig('States.png')

  plt.show()


plot()