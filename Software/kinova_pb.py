# -*- coding: utf-8 -*-
"""
Created on Tue May  1 17:42:03 2018

@author: jack
"""

from pybullet import *
import pybullet_data
from PID import *
from Rotations import *
import time
import csv
import math


class PYBULLET(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._timestep = 0.01
        
        self._simulator = "PyBullet"
        self._physics_engine = ""
        self._experiment = ""
        self._current_iteration = 0

        #Setup PyBullet
        self._physicsClient = connect(GUI)#or p.DIRECT for non-graphical version
        setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        setGravity(0,0,-9.8)
        configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)
        setPhysicsEngineParameter(fixedTimeStep = self._timestep)
        
        #Setup Floor
        self._planeId = loadURDF("plane.urdf")
                
        #Robot Arm
        self._kinovaStartPos = [0,0,0.055]
        self._kinova = loadURDF("kinova_description/urdf/m1n6s300.urdf",self._kinovaStartPos,useFixedBase=1)
        #Cube
        self._cubedim = [0.0375,0.0375,0.0375]
        self._cube_mass = 0.0884
        self._visualShapeId = -1
        self._cubeStartPos = [0.5,0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0,0,0])
        createMultiBody(0,0)
        self._colCubeId = createCollisionShape(GEOM_BOX,halfExtents=self._cubedim)
        self._cubeUid = createMultiBody(self._cube_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
        
    def set_current_iteration(self, iteration):
        self._current_iteration= iteration
        
    def set_experiment(self,experiment):
        self._experiment = experiment
        
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)
    
    def run_pybullet(self):
        self.set_num_steps()
        for simStep in range(self._num_steps):
            
            self._pid = set_target_thetas(self._num_steps, self._pid,self._experiment,self._simulator,simStep)
            
            
            if simStep % 5 == 0:
                for jointNum in range(7):
                    self._theta[jointNum] = getJointState(self._kinova, jointNum)[0]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    setJointMotorControl2(bodyIndex=self._kinova,jointIndex=jointNum,controlMode=VELOCITY_CONTROL,targetVelocity=self._linearVelocity[jointNum],force=2000)
                
#                positions.append([p.getLinkState(kinova,6,1)[0][0], p.getLinkState(kinova,6,1)[0][1],p.getLinkState(kinova,6,1)[0][2],p.getLinkState(kinova,6,1)[1][0],p.getLinkState(kinova,6,1)[1][1],p.getLinkState(kinova,6,1)[1][2],p.getLinkState(kinova,6,1)[1][3],p.getBasePositionAndOrientation(cubeUid)[0][0],p.getBasePositionAndOrientation(cubeUid)[0][1],p.getBasePositionAndOrientation(cubeUid)[0][2],p.getBasePositionAndOrientation(cubeUid)[1][0],p.getBasePositionAndOrientation(cubeUid)[1][1],p.getBasePositionAndOrientation(cubeUid)[1][2],p.getBasePositionAndOrientation(cubeUid)[1][3]])
    
    
            stepSimulation()
            time.sleep(0.01)
    
        disconnect()
    
        saveStats(self._experiment,self._current_iteration, self._physics_engine,self._simulator, self._positions)
