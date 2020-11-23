# SE(2)

The data generated for SE(2) comes from `system.py`, using `UnicycleSystem.gen_data()`. It's that a a "unicycle" model. 

## Model

The model is exactly (in the SE(2) Lie Group):

<!-- \begin{align*}
    X_{n+1} &= X_n
\begin{bmatrix}
\cos(\phi_n) & -\sin(\phi_n) & v_n \\
\sin(_n) & \cos(\phi_n) & 0 \\
0 & 0 & 1 \\
\end{bmatrix}  e^{\xi \string^} \\
Z_n &= X_n 
\begin{bmatrix}
 0\\ 0 \\ 1 
\end{bmatrix} + W_n
\end{align*} -->
<p align="center"><img src="https://latex.codecogs.com/gif.latex?%5Cbegin%7Balign*%7D%20X_%7Bn&plus;1%7D%20%26%3D%20X_n%20%5Cbegin%7Bbmatrix%7D%20%5Ccos%28%5Cphi_n%29%20%26%20-%5Csin%28%5Cphi_n%29%20%26%20v_n%20%5C%5C%20%5Csin%28_n%29%20%26%20%5Ccos%28%5Cphi_n%29%20%26%200%20%5C%5C%200%20%26%200%20%26%201%20%5C%5C%20%5Cend%7Bbmatrix%7D%20e%5E%7B%5Cxi%20%5Cstring%5E%7D%20%5C%5C%20Z_n%20%26%3D%20X_n%20%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%20%5C%5C%201%20%5Cend%7Bbmatrix%7D%20&plus;%20W_n%20%5Cend%7Balign*%7D"></p>


Where phi and v are the controls, and xi ~ N(0, Q), W_n ~ N(0, R). In regular coordinates (without noise), this looks like

<!-- \begin{align*}
    x_{n+1} &= x_n + v_n \cos(\theta) \\
    y_{n+1} &= y_n + v_n \sin(\theta) \\
    \theta_{n+1} &= \theta_n + \phi_n \\
    z_{xn} &= x_n \\
    z_{yn} &= y_n \\
\end{align*} -->
<p align="center"><img src="https://latex.codecogs.com/gif.latex?%5Cbegin%7Balign*%7D%20x_%7Bn&plus;1%7D%20%26%3D%20x_n%20&plus;%20v_n%20%5Ccos%28%5Ctheta%29%20%5C%5C%20y_%7Bn&plus;1%7D%20%26%3D%20y_n%20&plus;%20v_n%20%5Csin%28%5Ctheta%29%20%5C%5C%20%5Ctheta_%7Bn&plus;1%7D%20%26%3D%20%5Ctheta_n%20&plus;%20%5Cphi_n%20%5C%5C%20z_%7Bxn%7D%20%26%3D%20x_n%20%5C%5C%20z_%7Byn%7D%20%26%3D%20y_n%20%5C%5C%20%5Cend%7Balign*%7D"></p>


## Installation

First, if you'd like to make a conda environment, do it now. A simple command to make and activate a new environment is
```
conda create -n iekf python=3.8
conda activate iekf
```
Then to install all required dependencies (basically just numpy and scipy), run
```
pip install -r requirements.txt
```
To render latex images, we used https://www.codecogs.com/latex/eqneditor.php. Simply copy your equation in, then at the bottom, copy the "URL encoded" and put it into a html tag like this:
```html
<p align="center"><img src=""></p>
```

## File Structure

All code includes relevant docstrings in is formatted to lean on OOP. Feel free to view the code to get an understanding of what each file/object does and what function arguments are. The basic over is:
* `system.py` - class `UnicycleSystem`
    * `gen_data` - Used to generate data using SE(2) model
    * `f_lie` - The process model in SE(2)
    * `f_standard` - The process model in standard coordinates
    * `h` - Measurement model for SE(2) and standard coordinates
    * `F`, `H` - Jacobians of process/measurement model used for EKF
    * `carat` - Takes in an element of R^3 and puts it in se(2)
* `ekf.py` - class `ExtendedKalmanFilter`
    * `predict`, `update` - Performs prediction and update steps of the EKF
    * `iterate` - Iterates through prediction and update steps.

Also note, each file has a little code at the end that illustrates how/what you can do with each class.
