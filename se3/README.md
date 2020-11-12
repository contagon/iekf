# SE(3)

The data generated for SE(3) comes from holodeck (the original, not our fork). More data can be generated at anytime using the file `data_gen.py`.

## Installation

First, if you'd like to make a conda environment, do it now. A simple command to make and activate a new environment is
```
conda create -n iekf python=3.8
conda activate iekf
```
Then to install all required dependencies, run
```
pip install -r requirements.txt
```

## Data Generation

To start collecting data, simply run
```
python data_gen.py filename.npz
```
Where `filename.npz` is where the data will be saved. Note the first time things are run, it'll automatically download the required holodeck pacakges, which are ~3.5gb. 

The controls for the quadcopter are:
| Command | Forward Key | Backward Key|
|---|:---:|:---:|
| Upwards Thrust |  w | s |
| Roll Torque | y | u |
| Pitch Torque | h | j |
| Yaw Torque | n | m |

To quit, press `q` at anytime. Note that when starting the simulation, your quadcopter is on the ground, so you might need to give it a second or two to settle.