import numpy as np
from numpy.linalg import inv
from scipy.linalg import expm

class InvariantEKF:

    def __init__(self, system, mu_0, sigma_0):
        """The newfangled Invariant Extended Kalman Filter

        Args:
            system    (class) : The system to run the iEKF on. It will pull Q, R, f, h, F, H from this.
            mu0     (nxn array) : Initial starting point of system
            sigma0  (mxm array) : Initial covariance of system"""
        self.sys = system
        self.mus = [mu_0]
        self.sigmas = [sigma_0]
    
    # These 2 properties are helpers. Since our mus and sigmas are stored in a list
    # it can be a pain to access the last one. These provide "syntactic sugar"
    @property
    def mu(self):
        return self.mus[-1]

    @property
    def sigma(self):
        return self.sigmas[-1]

    def predict(self, u):
        """Runs prediction step of iEKF.

        Args:
            u       (k ndarray) : control taken at this step

        Returns:
            mu    (nxn ndarray) : Propagated state
            sigma (nxn ndarray) : Propagated covariances"""
        
        #get mubar and sigmabar
        mu_bar, U = self.sys.f_lie(self.mu, u)
        adj_U = self.sys.adjoint(U)
        sigma_bar = adj_U @ self.sigma @ adj_U.T + self.sys.Q * self.sys.deltaT**2

        #save for use later
        self.mus.append( mu_bar )
        self.sigmas.append( sigma_bar )

        return mu_bar, sigma_bar

    def update(self, z):
        """Runs correction step of iEKF.

        Args:
            z (m ndarray): measurement at this step

        Returns:
            mu    (nxn ndarray) : Corrected state
            sigma (nxn ndarray) : Corrected covariances"""
        H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, 0]])

        b = np.array([0, 0, 0, 1, 0])
        z = np.array([z[0], z[1], z[2], 0, 0])
        V = ( inv( self.mu )@z - self.sys.b )[:-2]

        invmu = inv(self.mu)[:3,:3]
        K = self.sigma @ H.T @ inv( H@self.sigma@H.T + invmu@self.sys.R@invmu.T )
        self.mus[-1] = self.mu @ expm( self.sys.carat(K @ V) )
        self.sigmas[-1] = (np.eye(9) - K @ H) @ self.sigma

        return self.mu, self.sigma

    def iterate(self, us, zs):
        """Given a sequence of observation, iterate through EKF
        
        Args:
            us (txk ndarray) : controls for each step, each of size k
            zs (txm ndarray) : measurements for each step, each of size m
            
        Returns:
            mus    (txnxn ndarray) : resulting states
            sigmas (txnxn ndarray) : resulting sigmas"""
        for u, z in zip(us, zs):
            self.predict(u)
            self.update(z)

        return np.array(self.mus)[1:], np.array(self.sigmas)[1:]


if __name__ == "__main__":
    def set_axes_equal(ax):
        """Set 3D plot axes to equal scale.

        Make axes of 3D plot have equal scale so that spheres appear as
        spheres and cubes as cubes.  Required since `ax.axis('equal')`
        and `ax.set_aspect('equal')` don't work on 3D.
        """
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ])
        min = limits[:,0].min()
        max = limits[:,1].max()
        ax.set_xlim3d([min, max])
        ax.set_ylim3d([min, max])
        ax.set_zlim3d([min, max])

    from system import QuadcopterSystem
    import matplotlib.pyplot as plt

    # setup system
    Q = np.diag([.001, .001, .001, 0, 0, 0, .001, .001, .001])
    R = np.diag([.001, .001, .001])
    filename = "ninety.npz"
    sys = QuadcopterSystem(filename, Q, R)
    t = sys.T

    # generate data from Lie Group method
    x, u, z = sys.gen_data(t, noise=True)

    # Run the iekf
    iekf = InvariantEKF(sys, x[0], np.eye(9)) # Assuming starting position is known?
    x_result, sigmas = iekf.iterate(u, z)

    # Plot results
    from mpl_toolkits.mplot3d import Axes3D

    plot = True
    if plot:
        # plot velocity
        t = np.arange(t)
        fig, ax = plt.subplots(1,3)
        ax[0].plot(t, x[:,0,3], label="Actual Velocity")
        ax[0].plot(t, x_result[:,0,3], label="Reconstructed Velocity")
        ax[1].plot(t, x[:,1,3], label="Actual Velocity")
        ax[1].plot(t, x_result[:,1,3], label="Reconstructed Velocity")
        ax[2].plot(t, x[:,2,3], label="Actual Velocity")
        ax[2].plot(t, x_result[:,2,3], label="Reconstructed Velocity")
        plt.legend()

        # plot position
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x[:,0,4], x[:,1,4], x[:,2,4], label="Actual")
        ax.plot(x_result[:,0,4], x_result[:,1,4], x_result[:,2,4], label="iEKF")
        ax.legend()
        set_axes_equal(ax)
        ax.set_box_aspect([1,1,1])
        plt.show()