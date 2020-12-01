import numpy as np
from scipy.linalg import expm, block_diag
from numpy.linalg import inv
np.set_printoptions(suppress=True,
   formatter={'float_kind':'{:0.5f}'.format}) 
import warnings

class QuadcopterSystem:

    def __init__(self, filename, Q, R):
        """Our system for a quadcopter, with IMU measurements as controls. 

        Args:
            filename  (str) : Where the recorded data has been stored
            Q (9,9 nparray) : Covariance of noise on state
            R (3x3 nparray) : Covariance of noise on measurements"""
        self.Q = Q
        self.R = R
        self.data = np.load(filename)
        self.deltaT = 1 / self.data['ticks']
        self.T = self.data['x'].shape[0]
        self.b = np.array([0, 0, 0, 1.0, 0])

    def gen_data(self, t, noise=True):
        """Generates model data using Lie Group model.

        Args:
            t      (int) : How many timesteps to run
            noise (bool) : Whether or not to include noise. Defaults to True.

        Returns:
            x (t,5,5 ndarray) : x steps after x0 
            u (t,2,3 ndarray) : controls that were applied
            z   (t,3 ndarray) : measurements taken.
        """
        if t > self.T:
            warnings.warn (f"You requested {t} steps, there's only {self.T} available.")
            t = self.T
        x = self.data['x'][:t]
        u = self.data['u'][:t]
        z = self.data['z'][:t]
        if noise:
            z += np.random.multivariate_normal(mean=np.zeros(3), cov=self.R, size=t)
            u[:,0] += np.random.multivariate_normal(mean=np.zeros(3), cov=self.Q[0:3,0:3], size=t)
            u[:,1] += np.random.multivariate_normal(mean=np.zeros(3), cov=self.Q[6:9,6:9], size=t)

        return x, u, z

    def f_lie(self, state, u, noise=False):
        """Propagates state forward in Lie Group. Used for IEKF.

        Args:
            state (5,5 ndarray) : X_n of model in Lie Group
            u     (2,3 ndarray) : U_n of model (IMU measurements)
            noise        (bool) : Whether or not to add noise. Defaults to False.

        Returns:
            X_{n+1} (5,5 ndarray)"""
        # transform u in to the right representation
        #get stuff we need
        Rinv = inv(state[:3, :3])
        v = state[:3, 3]
        g = np.array([0, 0, -9.81])

        ## put it together
        u_vec = np.zeros(9)
        #acceleration into vdot
        u_vec[0:3] = u[0,:] + Rinv@g
        #velocity into pdot
        u_vec[3:6] = Rinv@v
        #angular accel into Rdot
        u_vec[6:9] = u[1,:]

        # do actual propagating
        if noise:
            w = np.random.multivariate_normal(mean=np.zeros(9), cov=self.Q)
        else:
            w = np.zeros(9)
        U = expm( self.carat( (u_vec + w)*self.deltaT ) )
        return (state @ U), U

    def h(self, state, noise=False):
        """Calculates measurement given a state. Note that the result is
            the same if it's in standard or Lie Group form, so we simplify into
            one function.
            
        Args:
            state (3 ndarray or 3,3 ndarray) : Current state in either standard or Lie Group form
            noise                     (bool) : Whether or not to add noise. Defaults to False.

        Returns:
            Z_n (3 ndarray or 3,3 ndarray)"""
        #add noise if needed
        if noise:
            w = np.random.multivariate_normal(mean=np.zeros(3), cov=self.R)
        else:
            w = np.zeros(3)
        return state[:3,3] + w

    def f_standard(self, state, u, noise=False):
        """Propagates state forward in regular coordinates. Used for EKF.

        Args:
            state (9 ndarray): X_n of model in coordinates
            u   (2,3 ndarray): U_n of model in standard coordinates (IMU)
            noise     (bool) : Whether or not to add noise. Defaults to False.

        Returns:
            X_{n+1} (3 ndarray)"""
        raise NotImplementedError("Not implemented in standard coordinates yet.")

    def F(self, state, u):
        """Jacobian of system using standard coordinates. Used for EKF.

         Args:
            state (3 ndarray) : X_n of model in coordinates
            u     (3 ndarray) : U_n of model in standard coordinates

        Returns:
            df / dx (3,3 ndarray)"""
        raise NotImplementedError("Not implemented in standard coordinates yet.")

    def H(self, state):
        """Jacobian of measurement model using standard coordinates. Used for EKF.

         Args:
            state (3 ndarray) : X_n of model in coordinates
            u     (3 ndarray) : U_n of model in standard coordinates

        Returns:
            dh / dx (3,3 ndarray)"""
        raise NotImplementedError("Not implemented in standard coordinates yet.")

    @staticmethod
    def cross(x):
        """Moves a 3 vector into so(3)

        Args:
            x (3 ndarray) : Parametrization of Lie Algebra

        Returns:
            x( (3,3 ndarray) : Element of so(3)"""
        return np.array([[   0, -x[2],  x[1]],
                        [ x[2],     0, -x[0]],
                        [-x[1],  x[0],     0]])

    @staticmethod
    def carat(xi):
        """Moves a 9 vector to the Lie Algebra se_2(3).

        Args:
            xi (9 ndarray) : Parametrization of Lie algebra

        Returns:
            xi^ (5,5 ndarray) : Element in Lie Algebra se_2(3)"""
        w_cross = QuadcopterSystem.cross(xi[6:9])
        v       = xi[0:3].reshape(-1,1)
        p       = xi[3:6].reshape(-1,1)
        return np.block([[w_cross, v, p],
                         [np.zeros((2,5))]])

    @staticmethod
    def adjoint(xi):
        """Takes adjoint of element in SE_2(3)

        Args:
            xi (5,5 ndarray) : Element in Lie Group

        Returns:
            Ad_xi (9,9 ndarray) : Adjoint in SE_2(3)"""
        R = xi[:3,:3]
        v_cross = QuadcopterSystem.cross(xi[:3,3])
        p_cross = QuadcopterSystem.cross(xi[:3,4])
        zero    = np.zeros((3,3))
        return np.block([[   R, v_cross@R, p_cross@R],
                         [zero,         R,      zero],
                         [zero,      zero,         R]])

# we do our testing down here
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
        

    # setup system
    Q = np.diag([.001, .001, .001, 0, 0, 0, .001, .001, .001])
    R = np.diag([.001, .001, .001])
    filename = "ninety.npz"
    sys = QuadcopterSystem(filename, Q, R)
    t = sys.T

    # load data, and run it through our system to check they match (ish)
    x, u, z = sys.gen_data(t, noise=False)
    x_result = np.zeros((t, 5, 5))
    x_result[0] = x[0]
    for i, ui in enumerate(u):
        if i+1 == t:
            break
        x_result[i+1] = sys.f_lie(x_result[i], ui)


    import matplotlib.pyplot as plt
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
        ax.plot(x_result[:,0,4], x_result[:,1,4], x_result[:,2,4], label="Noisy")
        ax.legend()
        set_axes_equal(ax)
        ax.set_box_aspect([1,1,1])
        plt.show()
