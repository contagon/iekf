from system import QuadcopterSystem
from iekf import InvariantEKF
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    # setup system
    Q = np.diag([.01, .01, .01, 0, 0, 0, .01, .01, .01])
    R = np.diag([.01, .01, .01])
    # Specify filenames of datasets to use
    filenames = ["run1.npz", "run2.npz", "run3.npz"]

    # Make all the plots and titles
    fig = plt.figure(figsize=(10,5))
    ax1 = fig.add_subplot(131, projection='3d')
    ax2 = fig.add_subplot(132, projection='3d')
    ax3 = fig.add_subplot(133, projection='3d')
    fig.suptitle("SE(3) IEKF Simulations", y=0.78)
    ax1.set_title("(a)", y=0.98)
    ax2.set_title("(b)", y=0.98)
    ax3.set_title("(c)", y=0.98)
    ax = [ax1, ax2, ax3]

    for i in range(len(filenames)):
        # Create the system
        sys = QuadcopterSystem(filenames[i], Q, R)
        t = sys.T

        # generate data from Lie Group method
        x, u, z = sys.gen_data(t, noise=True)

        # Run the iekf
        iekf = InvariantEKF(sys, x[0], np.eye(9)) # Assuming starting position is known?
        x_result, sigmas = iekf.iterate(u, z)

        # Get the unfiltered positions
        x_unfilt = np.zeros((t, 5, 5))
        x_unfilt[0] = x[0]
        for j, ui in enumerate(u):
            if j+1 == t:
                break
            x_unfilt[j+1], U = sys.f_lie(x_unfilt[j], ui)

        # Plot the positions
        ax[i].plot(x[:,0,4], x[:,1,4], x[:,2,4], label="Actual")
        ax[i].plot(x_unfilt[:,0,4], x_unfilt[:,1,4], x_unfilt[:,2,4], label="Unfiltered")
        ax[i].plot(x_result[:,0,4], x_result[:,1,4], x_result[:,2,4], label="iEKF")
        # ax[i].legend()
    ax[0].legend()
    # plt.show()
    plt.savefig("figure.pdf", bbox_inches='tight')



