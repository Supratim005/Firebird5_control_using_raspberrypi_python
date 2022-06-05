import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd


plt.style.use('seaborn')
fig = plt.figure()
ax = fig.add_subplot(1,1,1)


def animation(i):
  actual = pd.read_csv("States.csv")
  target =pd.read_csv()
  x1 = []
  y1 = []
  x2 = []
  y2 = []

  x1 = actual[0:i]['x']
  y1 = actual[0:i]['y']

  x2 = target[0:i]['x']
  y2 = target[0:i]['y']

  ax.clear()
  ax.plot(x1, y1)
  ax.plot(x2, y2)

animation = FuncAnimation(fig, func=animation, interval=1000)
plt.show()