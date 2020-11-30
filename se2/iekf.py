import numpy as np
from numpy.linalg import inv
from scipy.linalg import expm

class InvariantEKF:

    def __init__(self, system, mu_0, sigma_0):
        """The newfangled Invariant Extended Kalman Filter

        Args:
            system    (class) : The system to run the iEKF on. It will pull Q, R, f, h, F, H from this.
            mu0     (ndarray) : Initial starting point of system
            sigma0  (ndarray) : Initial covariance of system"""
        self.sys = system
        #convert x0 into lie group if needed
        if mu_0.shape == (3,):
            mu_0 = np.array([[np.cos(mu_0[2]), -np.sin(mu_0[2]), mu_0[0]],
                             [np.sin(mu_0[2]), np.cos(mu_0[2]), mu_0[1]],
                             [0,             0,             1]])
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
        mu_bar = self.sys.f_lie(self.mu, u)
        adj_u = self.sys.adjoint( inv(expm(self.sys.carat( np.array([u[0], 0, u[1]])*self.sys.deltaT ))) )
        sigma_bar = adj_u @ self.sigma @ adj_u.T + self.sys.Q

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

        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        V = ( inv( self.mu )@z - self.sys.b )[:-1]

        invmu = inv(self.mu)[:2,:2]
        K = self.sigma @ H.T @ inv( H@self.sigma@H.T + invmu@self.sys.R@invmu.T )
        self.mus[-1] = self.mu @ expm( self.sys.carat(K @ V) )
        self.sigmas[-1] = (np.eye(3) - K @ H) @ self.sigma

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
    from system import UnicycleSystem
    import matplotlib.pyplot as plt

    # setup system
    Q = np.diag([.000001, .000001, .001])
    R = np.diag([.001, .001])
    dt = 0.1
    sys = UnicycleSystem(Q, R, dt)
    x0 = np.zeros(3)

    # generate data from Lie Group method
    t = 100
    u = lambda t: np.array([t/10, 1])
    x, _, z = sys.gen_data(x0, u, t, noise=True)

    # Run the iekf
    u = np.array([u(t) for t in range(t)])
    iekf = InvariantEKF(sys, x0, np.eye(3))
    mus, sigmas = iekf.iterate(u, z)

    # plot results
    plt.plot(x[:,0,2], x[:,1,2], label="Actual Location")
    plt.plot(z[:,0], z[:,1], label="Measurements", alpha=0.5)
    plt.plot(mus[:,0,2], mus[:,1,2], label="iEKF Results")
    plt.legend()
    plt.show()