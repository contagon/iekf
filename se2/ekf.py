import numpy as np
from numpy.linalg import inv

class ExtendedKalmanFilter:

    def __init__(self, system, mu_0, sigma_0):
        """The infamous Extended Kalman Filter

        Args:
            system    (class) : The system to run the EKF on. It will pull Q, R, f, h, F, H from this.
            mu0     (ndarray) : Initial starting point of system
            sigma0  (ndarray) : Initial covariance of system"""
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
        """Runs prediction step of EKF.

        Args:
            u (k ndarray): control taken at this step

        Returns:
            mu      (n ndarray) : Propagated mean
            sigma (nxn ndarray) : Propagated covariances"""
        #get mubar and sigmabar
        mu_bar = self.sys.f_standard(self.mu, u)
        F = self.sys.F(mu_bar, u)
        sigma_bar = F@self.sigma@F.T + self.sys.Q

        #save for use later
        self.mus.append( mu_bar )
        self.sigmas.append( sigma_bar )

        return mu_bar, sigma_bar

    def update(self, z):
        """Runs correction step of EKF.

        Args:
            z (m ndarray): measurement at this step

        Returns:
            mu      (n ndarray) : Corrected mean
            sigma (nxn ndarray) : Corrected covariances"""
        H = self.sys.H(self.mu)
        zbar = self.sys.h(self.mu)

        K = self.sigma@H.T@ inv( H@self.sigma@H.T + self.sys.R )
        self.mus[-1] = self.mu + K@(z - zbar)
        self.sigmas[-1] = (np.eye(3) - K@H)@self.sigma

        return self.mu, self.sigma

    def iterate(self, us, zs):
        """Given a sequence of observation, iterate through EKF
        
        Args:
            us (txk ndarray) : controls for each step, each of size k
            zs (txm ndarray) : measurements for each step, each of size m
            
        Returns:
            mus      (txn ndarray) : resulting means
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
    sys = UnicycleSystem(Q, R)
    x0 = np.zeros(3)

    # generate data from Lie Group method
    t = 100
    u = lambda t: np.array([1, np.sin(t/2)]) * dt
    x, _, z = sys.gen_data(x0, u, t, noise=True)
    #remove "1" from z
    z = z[:,:2]

    # Run the ekf
    u = np.array([u(t) for t in range(t)])
    ekf = ExtendedKalmanFilter(sys, x0, np.ones((3,3)))
    mus, sigmas = ekf.iterate(u, z)

    # plot results
    plt.plot(x[:,0,2], x[:,1,2], label="Actual Location")
    plt.plot(z[:,0], z[:,1], label="Measurements", alpha=0.5)
    plt.plot(mus[:,0], mus[:,1], label="EKF Results")
    plt.legend()
    plt.show()

