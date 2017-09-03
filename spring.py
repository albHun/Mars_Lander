# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
from pprint import pprint


G = 6.67408e-11
M = 6.42e23
k = G * M

"""
    Euler Method
"""

# mass, spring constant, initial position and velocity
m = 1
k = 1
position = np.array([[1.], [0.], [0.]])
velocity = np.zeros((3,1))

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
p_list = []
v_list = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    p_list.append(position)
    v_list.append(velocity)

    # calculate new position and velocity
    a = - k * position / np.linalg.norm(position, ord = 2)
    position = position + dt * velocity
    velocity = velocity + dt * a

# convert trajectory lists into arrays, so they can be indexed more easily
p_array = np.array([pos[0] for pos in p_list])
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.subplot(211)
# plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, p_array, label='x (m)')
# plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()




"""
    Verlet Method
"""

# mass, spring constant, initial position and velocity
m = 1
k = 1
position = np.array([[1000.], [0.], [0.]])
velocity = np.array([[0.], [50000.], [0.]])

# simulation time, timestep and time
t_max = 100
dt = 0.001
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
p_list = []
v_list = []

# initialize x(-dt)
previous_p = position - dt * velocity

# Euler integration
for t in t_array:

    p_list.append(position)
    v_list.append(velocity)
    
    # Calculate the next x and v
    a = - k * position / m
    next_p = 2. * position - previous_p + dt ** 2. * a
    next_v = 1. / dt * (next_p - position)


    # Propagate
    previous_p = position
    position = next_p
    v = next_v

# convert trajectory lists into arrays, so they can be indexed more easily
x_array = [pos[0][0] for pos in p_list]
y_array = [pos[1][0] for pos in p_list]
v_array = np.array(v_list)

pprint(p_array)

# plot the position-time graph
plt.subplot(212)
# plt.clf()
plt.xlabel('(m)')
plt.grid()
plt.plot(x_array, y_array)
# plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()




plt.show()


# The critical dt value to make the Verlet method unstable is 2