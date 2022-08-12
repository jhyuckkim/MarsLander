# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

m = 1
M = 6.42e23
G = 6.67e-11
r = np.array([2396384, 2396384, 0])
dr = np.array([3000, -3000, 0])

# simulation time, timestep and time
t_max = 10000
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = []
dr_list = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    r_list.append(r)
    dr_list.append(dr)

    # calculate new position and velocity
    ddr = (- G * M / (np.linalg.norm(r) ** 3)) * r
    r = r + dt * dr
    dr = dr + dt * ddr

# Create list for x, y and z coordinates and append following values from r_list
    x_list = []
    y_list = []
    z_list = []
for i in range(len(r_list)):
    x_list.append(r_list[i][0])
    y_list.append(r_list[i][1])
    z_list.append(r_list[i][2])
x_array = np.array(x_list)
y_array = np.array(y_list)
z_array = np.array(z_list)

# # plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('x')
plt.ylabel('y')
plt.grid()
plt.plot(x_array, y_array)
plt.legend()
# plt.show()

def plot_mars_trajectory(m, r, dr, t_max, dt):
    M = 6.42e23
    G = 6.67e-11
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    r_list = []
    dr_list = []

    # Verlet integration
    for i in range(len(t_array)):
        if i == 0:
            r_list.append(r)
            dr_list.append(dr)

            ddr = (- G * M / (np.linalg.norm(r) ** 3)) * r
            r = r + dt * dr
            dr = dr + dt * ddr
        else:
            r_list.append(r)
            dr_list.append(dr)

            ddr = (- G * M / (np.linalg.norm(r) ** 3)) * r
            r = 2 * r - r_list[i - 1] + ddr * (dt ** 2)
            dr = (r - r_list[i]) / dt

    # Create list for x, y and z coordinates and append following values from r_list
        x_list = []
        y_list = []
        z_list = []
    for i in range(len(r_list)):
        x_list.append(r_list[i][0])
        y_list.append(r_list[i][1])
        z_list.append(r_list[i][2])
    x_array = np.array(x_list)
    y_array = np.array(y_list)
    z_array = np.array(z_list)

    # plot the x-y graph
    plt.figure(1)
    plt.clf()
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    plt.plot(x_array, y_array)
    plt.legend()
    plt.show()

def plot_mars_altitude(m, r, dr, t_max, dt):
    M = 6.42e23
    G = 6.67e-11
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    r_list = []
    dr_list = []

    # Verlet integration
    for i in range(len(t_array)):
        if i == 0:
            r_list.append(r)
            dr_list.append(dr)

            ddr = (- G * M / (np.linalg.norm(r) ** 3)) * r
            r = r + dt * dr
            dr = dr + dt * ddr
        else:
            r_list.append(r)
            dr_list.append(dr)

            ddr = (- G * M / (np.linalg.norm(r) ** 3)) * r
            r = 2 * r - r_list[i - 1] + ddr * (dt ** 2)
            dr = (r - r_list[i]) / dt

    # Create list for x, y and z coordinates and append following values from r_list
        al_list = []
    for i in range(len(r_list)):
        al_list.append(np.linalg.norm(r_list[i]))
    al_array = np.array(al_list)

    # Plot altitude-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time')
    plt.ylabel('altitude')
    plt.grid()
    plt.plot(t_array, al_array)
    plt.legend()
    plt.show()

# Straight down descent altiude plot
r = np.array([3400000, 0, 0])
dr = np.array([0, 0, 0])
plot_mars_altitude(1, r, dr, 1000, 1)

# Circular orbit
plot_mars_trajectory(1, np.array([3400000, 0, 0]), np.array([0, -3548.8772, 0]), 10000, 1)

# Eliptical orbit
plot_mars_trajectory(1, np.array([3400000, 0, 0]), np.array([0, -4500, 0]), 100000, 1)

# Hyperbolic escape
plot_mars_trajectory(1, np.array([3400000, 0, 0]), np.array([0, -5020, 0]), 100000, 1)