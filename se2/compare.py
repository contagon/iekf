import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from system import UnicycleSystem
from iekf import InvariantEKF
from ekf import ExtendedKalmanFilter

def shift_to_final(goal, current):
    if current[-1] < goal-np.pi:
        current += 2*np.pi
    elif current[-1] > goal+np.pi:
        current -= 2*np.pi
    return current

# setup system
Q = np.diag([.001, 0, .1])
R = np.diag([.001, .001])
dt = 0.1
sys = UnicycleSystem(Q, R, dt)
x0 = np.zeros(3)

# generate data from Lie Group method
t = 50
u = lambda t: np.array([t/60, 1])
x, u, z = sys.gen_data(x0, u, t, noise=True)

xaxis = np.arange(t) * dt
iterations = range(-20, 21)
angle_increment = 2*np.pi / (len(iterations) - 1)
angle = -np.pi

angle_final = -np.unwrap(np.arctan2(x[:,0,1], x[:,0,0]))[-1]

# make all plots and titles
sns.set(font_scale=0.85)
fig, ax = plt.subplots(2,5, figsize=(10,5))
gs = ax[0,3].get_gridspec()
ax[0,3].remove()
ax[1,3].remove()
ax[0,4].remove()
ax[1,4].remove()
axbig = fig.add_subplot(gs[0:,-2:])
fig.suptitle("SE(2) Filter Comparisons")
ax[0,0].set_title('x')
ax[0,1].set_title('y')
ax[0,2].set_title('theta')
state = [r"$x$", r"$y$", r"$\theta$"]
algo = ["EKF", "IEKF"]
for i in range(3):
    for j in range(2):
        ax[j,i].set_title(f"{algo[j]}: {state[i]}")
        # TODO: Do these need labels?
        # if i == 0:
        #     ax[j,i].set_ylabel("Meters")
        # if i == 2:
        #     ax[j,i].set_ylabel("Radians")
        if j == 1:
            ax[j,i].set_xlabel("Seconds")

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
    # mus_ekf[:,2] = (mus_ekf[:,2] + np.pi) % (2 * np.pi) - np.pi # Convert angles to be between -pi and pi
    mus_ekf[:,2] = shift_to_final(angle_final, np.unwrap(mus_ekf[:,2]))

    # plot x y and thetas
    ax[0,0].plot(xaxis, mus_ekf[:,0], '--')
    ax[0,1].plot(xaxis, mus_ekf[:,1], '--')
    ax[0,2].plot(xaxis, mus_ekf[:,2], '--')

    ax[1,0].plot(xaxis, mus_iekf[:,0,2], '--')
    ax[1,1].plot(xaxis, mus_iekf[:,1,2], '--')
    ax[1,2].plot(xaxis, shift_to_final(angle_final, -np.unwrap(np.arctan2(mus_iekf[:,0,1], mus_iekf[:,0,0]))), '--')

# plot true states
ax[0,0].plot(xaxis, x[:,0,2], 'k')
ax[0,1].plot(xaxis, x[:,1,2], 'k')
ax[0,2].plot(xaxis, -np.unwrap(np.arctan2(x[:,0,1], x[:,0,0])), 'k')

ax[1,0].plot(xaxis, x[:,0,2], 'k')
ax[1,1].plot(xaxis, x[:,1,2], 'k')
ax[1,2].plot(xaxis, -np.unwrap(np.arctan2(x[:,0,1], x[:,0,0])), 'k')

# shrink tick labels
# TODO Does this need to happen?
# for i in range(2):
#     for j in range(3):
#         print(i,j)
#         ax[i,j].set_xticklabels(ax[i,j].get_xticks(), size=10)
#         ax[i,j].set_yticklabels(ax[i,j].get_yticks(), size=10)

# plot trajectories of the last run
axbig.set_title('Example Trajectory')
axbig.plot(x[:,0,2], x[:,1,2], label="Actual Location")
axbig.plot(z[:,0], z[:,1], label="Measurements", alpha=0.5)
axbig.plot(mus_ekf[:,0], mus_ekf[:,1], label="EKF Results")
axbig.plot(mus_iekf[:,0,2], mus_iekf[:,1,2], label="iEKF Results")
axbig.legend()

plt.subplots_adjust(top=0.893,
bottom=0.091,
left=0.061,
right=0.992,
hspace=0.283,
wspace=0.292)
# plt.show()

plt.savefig('figure.pdf', bbox_inches='tight')