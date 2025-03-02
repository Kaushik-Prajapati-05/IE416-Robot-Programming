import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('a_results.csv')
print(df.columns)

time = np.array(df['Time'])
dist_to_obs = np.array(df['Distance to Obstacle'])
dist_to_goal = np.array(df['Distance to Goal'])

plt.figure(figsize=(8, 6))
plt.plot(time, dist_to_obs, color='red', label='NEO: Distance to Obstacle')
plt.plot(time, dist_to_goal, color='blue', label='NEO: Distance to Goal')
plt.ylim((0, 0.6))
plt.xlabel('time')
plt.ylabel('Distance')
plt.title('Experiment - 1a')
plt.grid()
plt.legend()
plt.savefig('a_plot.png')
plt.show()