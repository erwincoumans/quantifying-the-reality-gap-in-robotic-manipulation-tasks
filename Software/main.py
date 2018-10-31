# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 12:01:49 2018

@author: jack
"""
from kinova_mj import *
from kinova_pb import *
from kinova_vr import *
from Rotations import *
          
experiment = "Single"          
#experiment = "Double"
#experiment = "Cube"
repeats = 20
            
if __name__ == '__main__':
    
    for iteration in range(repeats):
        simulate = MUJOCO()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.run_mujoco()
    
    for iteration in range(repeats):
        simulate = PYBULLET()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.run_pybullet()
        
    open_vrep()
    for iteration in range(repeats):
        simulate = VREP()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.Bullet283()
    for iteration in range(repeats):
        simulate = VREP()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.Bullet278()
    for iteration in range(repeats):
        simulate = VREP()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.ODE()
    for iteration in range(repeats):
        simulate = VREP()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.Vortex()
    for iteration in range(repeats):
        simulate = VREP()
        simulate.set_current_iteration(iteration)
        simulate.set_experiment(experiment)
        simulate.Newton()