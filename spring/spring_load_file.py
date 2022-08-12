import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/spring/trajectories2.txt')
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend()
plt.show()
