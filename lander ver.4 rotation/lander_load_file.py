import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/lander/assignment5.txt')

altitude = results[:, 0]
Kh = 0.005
target = -(0.5 + Kh * altitude)

Kh2 = 0.02
target2 = -(0.5 + Kh2 * altitude)

Kh3 = 0.001
target3 = -(0.5 + Kh3 * altitude)

plt.figure(1)
plt.clf()
plt.xlabel('altitude')
plt.ylabel('descending rate')
plt.grid()
plt.plot(altitude, -results[:, 1], label='simulation')
plt.plot(altitude, -target, label='target')
plt.plot(altitude, -target2, label='target2')
plt.plot(altitude, -target3, label='target3')
plt.plot(altitude, np.log(0.02/600*altitude+1)*600, label='new')
plt.xlim(xmin=0, xmax=2.5e06)
plt.ylim(ymin=0, ymax=6000)
plt.legend()
plt.show()

