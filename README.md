# Quantifying the Reality Gap in Robotic Manipulation Tasks

This Repository contains both the raw data generated from our experiments and the software used to create the simulated data.

## Paper

**Quantifying the Reality Gap in Robotic Manipulation Tasks**

*Jack Collins, [David Howard](https://people.csiro.au/H/D/David-Howard), [Jürgen Leitner](http://juxi.net)*

## Installation

This code was developed with Python 3.5 on Ubuntu 16.04.

V-REP: Version 3.5.0*

PyBullet: Version 2.1.1

MuJoCo: Version 1.5.0

*Please add vrep.py, vrepConst.py and remoteApi.so files relevant for your distribution of V-Rep to the Software directory.

## Data and Files

Data Directory:
1. MotionCapture directory contains the ground truth data in tsv files (without headings) for all three experiments.
2. Simulation directory contains the simulated data in csv files (without headings) for all three experiments.
3. MC_Data_Format.tsv contains a raw data file with headings and additional relevant data output from Qualisys Track Manager software.
4. Sim_Data_Format.csv contains a raw data file with headings describing the simulated data (this is relevant for all simulated data except for PyBullet which exports rotations as [w x y z]).

Software Directory:
1. Results subdirectory is where the simulated results are saved.
2. kinova_description cantains the meshes, binary scene description for V-Rep, URDF and Mujoco XML to describe the Kinova Mico2 robot arm.
3. Python script and modules.

## Running

1. Add vrep.py, vrepConst.py and remoteApi.so files relevant for your distribution of V-Rep to the Software directory.
2. Uncomment the experiment you want to run in main.py.
3. Run main.py



**Contact**

Any questions or comments contact [Jack Collins](mailto:j30.collins@hdr.qut.edu.au).
