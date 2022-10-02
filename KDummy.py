import pybullet as p, numpy as np
import time

import pybullet_data

p.connect(p.GUI, options='--background_color_red=0.70 --background_color_green=1.0 --background_color_blue=1')
p.setAdditionalSearchPath(pybullet_data.getDataPath())
thefloor=p.loadSDF("plane_stadium.sdf")
# kale = p.loadURDF('/home/admin/mujokaleido/KSTL.urdf'
kale = p.loadMJCF('/home/admin/mujokaleido/KSTL26.xml')
        # , basePosition=[0, 0, 1]
        # , useFixedBase=1
        # ,flags = p.URDF_MERGE_FIXED_LINKS)

# kale = p.loadURDF("/home/pybullet_robots/data/cassie/urdf/cassie_with_torso.urdf", basePosition=[0, 0, 0], useFixedBase=1)

gravId = p.addUserDebugParameter("gravity", -10, 10, -9.8)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=1)
p.changeDynamics(kale, -1, linearDamping=1, angularDamping=1)
# print("LOOOOOOW")
for j in range(p.getNumJoints(kale)):
  lo, hi= p.getJointInfo(kale,j)[8], p.getJointInfo(kale, j)[9]

  # p.changeDynamics(kale, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(kale, j)
 

  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), lo, hi, 0))
  print(",",info)
p.setRealTimeSimulation(1)


while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  chestY_state= p.getLinkState(1,0); waistState= p.getBasePositionAndOrientation(1)
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(kale, jointIds[i], p.POSITION_CONTROL, targetPos)#, force=5 * 240.)
  rollRad, pitchRad= p.getEulerFromQuaternion(chestY_state[1])[0], p.getEulerFromQuaternion(chestY_state[1])[1]
  rollDeg, pitchDeg = np.rad2deg(rollRad), np.rad2deg(pitchRad)
  waistAngle = waistState[1]
  waistEuler= p.getEulerFromQuaternion(waistAngle)
  waistRollRad= waistEuler[0]
  waistPitchRad= waistEuler[1]   

  # print("Chest Y ROLL", int(rollDeg), " current PIIIT", int(pitchDeg))
  # print("WAIST ROLL", int(np.rad2deg(waistRollRad)), " WAIST PIIIT", int(np.rad2deg(waistPitchRad)))
    # if abs(pitchDeg) > 45: print("PIIIIITCH")    
  time.sleep(0.025)
