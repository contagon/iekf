# SE(2)

The data generated for SE(2) comes from `system.py`, using `UnicycleSystem.gen_data()`. It's that a a "unicycle" model. 

## Model

The model is exactly (in the SE(2) Lie Group):

<p align="center"><img src="https://rawgit.com/in	git@github.com:contagon/iekf/master/se2/svgs/fff493ae8f8d9b9068e315eddac1b141.svg?invert_in_darkmode" align=middle width=302.2383606pt height=124.93263584999998pt/></p>

Where phi and v are the controls, and xi ~ N(0, Q), W_n ~ N(0, R). In regular coordinates (without noise), this looks like

<p align="center"><img src="https://rawgit.com/in	git@github.com:contagon/iekf/master/se2/svgs/459bf179f13d4081f8c7e9d9817d3415.svg?invert_in_darkmode" align=middle width=157.9623375pt height=115.66207785pt/></p>

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
To render the latex in the README for github, you'll need latex, dvisvgm, and geometry installed on your system. To install, then compile run
```
sudo apt install texlive-latex-base texlive-extra-utils
python -m readme2tex --output README.md README_MATH.md
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
