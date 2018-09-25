# -*- coding: utf-8 -*-
"""
Created on Mon May 28 14:01:53 2018

@author: jack
"""

import vrep
import sys
from PID import *
from Rotations import *
import csv
import math

from PID import *

simulator = "VRep"
physics_engine = "Bullet283"
experiment = "Single"
iteration = 0

class VREP(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._timestep = 0.05
        self._arm_joints = [0,0,0,0,0,0,0,0,0]
        self._cube = 0
        
        self._simulator = "VRep"
        self._physics_engine = ""
        self._experiment = ""
        self._current_iteration = 0

        vrep.simxFinish(-1) # just in case, close all opened connections
        self._clientID=vrep.simxStart('127.0.0.1',19997,True,False,5000,5) # Connect to V-REP
        
        vrep.simxLoadScene(self._clientID, "kinova_description/urdf/Kinova5.ttt", 1, vrep.simx_opmode_blocking)
        
        errorCode,self._arm_joints[0] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_1', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[1] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_2', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[2] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_3', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[3] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_4', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[4] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_5', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[5] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_6', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[6] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_1', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[7] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_2', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[8] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_3', vrep.simx_opmode_blocking)
        errorCode,self._cube = vrep.simxGetObjectHandle(self._clientID, 'Cuboid', vrep.simx_opmode_blocking)
        
    def set_current_iteration(self, iteration):
        self._current_iteration= iteration
        
    def set_experiment(self,experiment):
        self._experiment = experiment
        
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep) 
        
    def Bullet283(self):
        if self._clientID!=-1:
            self._physics_engine = "Bullet283"
            self.run_simulation()
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
        
    def Bullet278(self):
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,0,vrep.simx_opmode_blocking)
            self._physics_engine = "Bullet278"
            self.run_simulation()
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
    
    def ODE(self):
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,1,vrep.simx_opmode_blocking)
            self._physics_engine = "ODE"
            self.run_simulation()
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
        
    def Vortex(self):
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,2,vrep.simx_opmode_blocking)
            self._physics_engine = "Vortex"
            self.run_simulation()
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
    
    def Newton(self):
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,3,vrep.simx_opmode_blocking)
            self._physics_engine = "Newton"
            self.run_simulation()
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
             
    def run_simulation(self):
        self.set_num_steps()
                    
        vrep.simxSynchronous(self._clientID,True);
        vrep.simxStartSimulation(self._clientID,vrep.simx_opmode_oneshot);            
        
        for simStep in range (self._num_steps):
            
            self._pid = set_target_thetas(self._num_steps, self._pid,self._experiment,self._simulator,simStep)
                   
            for joint in range(6):
                errorCode, self._theta[joint] = vrep.simxGetJointPosition(self._clientID,self._arm_joints[joint],vrep.simx_opmode_blocking)
                self._linearVelocity[joint] = self._pid[joint].get_velocity(math.degrees(self._theta[joint]))/self._convertdeg2rad
                vrep.simxSetJointTargetVelocity(self._clientID,self._arm_joints[joint],self._linearVelocity[joint],vrep.simx_opmode_oneshot)
            vrep.simxSynchronousTrigger(self._clientID)

            vrep.simxGetPingTime(self._clientID)
            
            self._positions.append(vrep.simxGetObjectPosition(self._clientID,self._arm_joints[5],-1,vrep.simx_opmode_blocking)[1] + vrep.simxGetObjectOrientation(self._clientID,self._arm_joints[5],-1,vrep.simx_opmode_blocking)[1] + vrep.simxGetObjectPosition(self._clientID,self._cube,-1,vrep.simx_opmode_blocking)[1] + vrep.simxGetObjectQuaternion(self._clientID,self._cube,-1,vrep.simx_opmode_blocking)[1])
            
    
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_blocking)
        vrep.simxFinish(self._clientID)

        saveStats(self._experiment,self._current_iteration, self._physics_engine,self._simulator, self._positions)

    
