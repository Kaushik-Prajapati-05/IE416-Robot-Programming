import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('c_results.csv')
print(df.columns)

time = np.array(df['time'])
dist_to_goal = np.array(df['neo_distance_to_goal'])
dist_to_obs1 = np.array(df['neo_distance_to_obstacle1'])
dist_to_obs2 = np.array(df['neo_distance_to_obstacle2'])

plt.figure(figsize=(8, 6))
plt.plot(time, dist_to_obs1, color='red', label='NEO: Distance to Obstacle-1')
plt.plot(time, dist_to_obs2, color='red', label='NEO: Distance to Obstacle-1')
plt.plot(time, dist_to_goal, color='blue', label='NEO: Distance to Goal')
plt.ylim((0, 0.6))
plt.xlabel('time')
plt.ylabel('Distance')
plt.title('Experiment - 1c')
plt.grid()
plt.legend()
plt.savefig('c_plot.png')
plt.show()