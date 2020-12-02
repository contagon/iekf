import numpy as np
import matplotlib.pyplot as plt

from system import UnicycleSystem
from iekf import InvariantEKF
from ekf import ExtendedKalmanFilter

# setup system
Q = np.diag([.001, 0, .1])
R = np.diag([.001, .001])
dt = 0.1
sys = UnicycleSystem(Q, R, dt)
x0 = np.zeros(3)

# generate data from Lie Group method
t = 50
u = lambda t: np.array([1, np.sin(t/2)])
u = lambda t: np.array([t, 1])
x, u, z = sys.gen_data(x0, u, t, noise=True)

xaxis = range(t) + np.ones(t)
iterations = range(-20, 21)
angle_increment = 2*np.pi / (len(iterations) - 1)
angle = -np.pi

fig1, ax1 = plt.subplots(1,3)
fig1.suptitle('EKF')
ax1[0].set_title('x')
ax1[1].set_title('y')
ax1[2].set_title('theta')
fig2, ax2 = plt.subplots(1,3)
fig2.suptitle('Invariant EKF')
for i in iterations:
    # Approximate the initial state
    xs = np.array([i, i, angle])
    angle += angle_increment

    # Run the iekf
    iekf = InvariantEKF(sys, xs, np.eye(3))
    mus_iekf, sigmas = iekf.iterate(u, z)

    # Run the ekf
    z_trunc = z[:,:2]
    ekf = ExtendedKalmanFilter(sys, xs, np.eye(3))
    mus_ekf, sigmas = ekf.iterate(u, z_trunc)
    mus_ekf[:,2] = (mus_ekf[:,2] + np.pi) % (2 * np.pi) - np.pi # Convert angles to be between -pi and pi

    # plot x y and thetas
    ax1[0].plot(xaxis, mus_ekf[:,0], '--')
    ax1[1].plot(xaxis, mus_ekf[:,1], '--')
    ax1[2].plot(xaxis, mus_ekf[:,2], '--')

    ax2[0].plot(xaxis, mus_iekf[:,0,2], '--')
    ax2[1].plot(xaxis, mus_iekf[:,1,2], '--')
    ax2[2].plot(xaxis, np.arctan2(mus_iekf[:,0,0], mus_iekf[:,0,1]), '--')

ax1[0].plot(xaxis, x[:,0,2], 'k')
ax1[1].plot(xaxis, x[:,1,2], 'k')
ax1[2].plot(xaxis, np.arctan2(x[:,0,0], x[:,0,1]), 'k')

ax2[0].plot(xaxis, x[:,0,2], 'k')
ax2[1].plot(xaxis, x[:,1,2], 'k')
ax2[2].plot(xaxis, np.arctan2(x[:,0,0], x[:,0,1]), 'k')

# plot trajectories of the last run
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('Trajectories')
ax.plot(x[:,0,2], x[:,1,2], label="Actual Location")
ax.plot(z[:,0], z[:,1], label="Measurements", alpha=0.5)
ax.plot(mus_ekf[:,0], mus_ekf[:,1], label="EKF Results")
ax.plot(mus_iekf[:,0,2], mus_iekf[:,1,2], label="iEKF Results")
ax.legend()

plt.show()