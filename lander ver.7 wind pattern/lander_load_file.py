import numpy as np
import matplotlib.pyplot as plt
sn1 = np.loadtxt('/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/lander ver.7 wind pattern/scenario1.txt')
sn5 = np.loadtxt('/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/lander ver.7 wind pattern/scenario5.txt')

altitude1 = sn1[:, 0]
altitude5 = sn5[:, 0]
Kh = 0.04
low = -(0.5 + Kh * altitude5)
high = -(0.5 + 0.02 * altitude5)

plt.figure(1)
plt.clf()
plt.xlabel('altitude')
plt.ylabel('descending rate')
plt.grid()
plt.plot(altitude1, -sn1[:, 1], label='Scenario 1')
plt.plot(altitude5, -sn5[:, 1], label='Scenario 5')
plt.plot(altitude5, -low, label='Linear model Kh = 0.04')
plt.plot(altitude5, -high, label='Linear model Kh = 0.02')
plt.plot(altitude5, np.log(0.04/800*altitude5+1)*800+0.5, label='Logarithmic model')
plt.legend(loc='lower right')
plt.show()

