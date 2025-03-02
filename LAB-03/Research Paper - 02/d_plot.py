import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv('d1_results.csv')
df2 = pd.read_csv('d2_results.csv')
df3 = pd.read_csv('d3_results.csv')

time1 = np.array(df1['time'])
time2 = np.array(df2['time'])
time3 = np.array(df3['time'])
m1 = np.array(df1['manipulability'])
m2 = np.array(df2['manipulability'])
m3 = np.array(df3['manipulability'])


plt.figure(figsize=(8, 6))
plt.plot(time1, m1, color='red', label='Trajectory a')
plt.plot(time2, m2, color='blue', label='Trajectory b')
plt.plot(time3, m3, color='purple', label='Trajectory c')
# plt.ylim((0, 0.6))
plt.xlabel('time')
plt.ylabel('Manipulability')
plt.title('Experiment - 1 using NEO')
plt.grid()
plt.legend()
plt.savefig('d_plot.png')
plt.show()

