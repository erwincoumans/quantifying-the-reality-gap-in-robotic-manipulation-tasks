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

kv=[10,40,10,4,4,2]

class PYBULLET(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._timestep = 0.0001
        
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
        resetBasePositionAndOrientation(self._kinova,self._kinovaStartPos,[0,0,0,1])
        for j in range (getNumJoints(self._kinova)):
          #print("joint[",j,"] jointName:", getJointInfo(self._kinova, j)[1], "jointType:", jointTypeNames[getJointInfo(self._kinova, j)[2]])
          force = 2000
          if j >=0 and j < 6:
            force = 0
          changeDynamics(self._kinova, j, linearDamping=0, angularDamping=0)
          setJointMotorControl2(self._kinova,j,POSITION_CONTROL, targetPosition=0, targetVelocity=0, force=force)
        #Cube
        self._cubedim = [0.0375,0.0375,0.0375]
        self._cube_mass = 0.0884
        self._visualShapeId = -1
        self._cubeStartPos = [0.5,0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0,0,0])
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
        pdTargetVel=[0,0,0,0,0,0]
        for simStep in range(self._num_steps):
            
            self._pid = set_target_thetas(self._num_steps, self._pid,self._experiment,self._simulator,simStep)
             
            if simStep % 500 == 0:
                for jointNum in range(6):
                    self._theta[jointNum] = getJointState(self._kinova, jointNum+2)[0]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    state = getLinkState(self._kinova,6,1)
                    pdTargetVel[jointNum]=self._linearVelocity[jointNum]
                self._positions.append([state[0][0], state[0][1],state[0][2],state[1][0],state[1][1],state[1][2],state[1][3],getBasePositionAndOrientation(self._cubeUid)[0][0],getBasePositionAndOrientation(self._cubeUid)[0][1],getBasePositionAndOrientation(self._cubeUid)[0][2],getBasePositionAndOrientation(self._cubeUid)[1][0],getBasePositionAndOrientation(self._cubeUid)[1][1],getBasePositionAndOrientation(self._cubeUid)[1][2],getBasePositionAndOrientation(self._cubeUid)[1][3]])
            setJointMotorControlArray(bodyIndex=self._kinova,jointIndices=[2,3,4,5,6,7],controlMode=PD_CONTROL,positionGains=[0,0,0,0,0,0], targetVelocities=pdTargetVel,velocityGains=kv, forces=[2000,2000,2000,2000,2000,2000])
            #for jointNum in range(6):
            #  setJointMotorControl2(bodyIndex=self._kinova,jointIndex=jointNum+2,controlMode=PD_CONTROL,positionGain=0, targetVelocity=pdTargetVel[jointNum],velocityGain=kv[jointNum], force=2000)

            stepSimulation()
            #time.sleep(self._timestep)
    
        disconnect()
    
        saveStats(self._experiment,self._current_iteration, self._physics_engine,self._simulator, self._positions)
