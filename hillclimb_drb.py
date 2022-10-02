import pybullet as p, time, pybullet_data, matplotlib.pyplot as plt, numpy as np
# from pdControllerStable import PDControllerStable

p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1')
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p. setGravity(0,0,-10)
# krobot = p.loadURDF("/home/admin/dribble_repo/RHP5Emod2.urdf",[0,0,0.99], useFixedBase=0, globalScaling=1, flags=p.URDF_USE_SELF_COLLISION |p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)# | p.URDF_MERGE_FIXED_LINKS)
krobot = p.loadURDF("/home/admin/dribble_repo/RHP5E_late.urdf",[0,0,1], useFixedBase=True, globalScaling=1 
,flags=p.URDF_USE_SELF_COLLISION
# |p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARE`NTS 
| p.URDF_MERGE_FIXED_LINKS
)

BODY_LINK_IDS = tuple(range(-1, 4))# + (16, 17))
# ARM4BODY_LINK_IDS = tuple((4, 17) + tuple(range(18, 27)))
L_ARM_LINK_IDS = tuple(range(4, 12))
R_ARM_LINK_IDS = tuple(range(12, 20))
L_LEG_LINK_IDS = tuple(range(20, 26))
R_LEG_LINK_IDS = tuple(range(46, 31))
all_link_ids = tuple(range(-1,33))

def setup_collision_group(link_a_ids, link_b_ids, flag):
    [setup_collision(i, j, flag) for i in link_a_ids for j in link_b_ids]

def setup_collision(link_a, link_b, flag):
    if link_a != link_b: p.setCollisionFilterPair(0, 0, link_a, link_b, flag)


# setup_collision_group(all_link_ids, all_link_ids, False)
# setup_collision_group(L_ARM_LINK_IDS, BODY_LINK_IDS, True)
# setup_collision_group(R_ARM_LINK_IDS, BODY_LINK_IDS, True)
# setup_collision_group(R_ARM_LINK_IDS, L_ARM_LINK_IDS, True)
# setup_collision_group(R_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
# setup_collision_group(R_ARM_LINK_IDS, L_LEG_LINK_IDS, True)
# setup_collision_group(L_ARM_LINK_IDS, L_LEG_LINK_IDS, True)    
# setup_collision_group(L_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
# setup_collision_group(L_LEG_LINK_IDS, R_LEG_LINK_IDS, True)

chestY_id, chestP_id= 0, 1
headY_id, headP_id= 2, 3

leftSP, leftSR, leftSY= 4, 5, 6
rightSP, rightSR, rightSY= 12, 13, 14
leftEP, leftEY= 7, 8
rightEP, rightEY= 15, 16
leftWP, leftWY, leftH= 9, 10, 11
rightWP, rightWY, rightH= 17, 18, 19

leftCY, leftCR, leftCP= 20, 21, 22
rightCY, rightCR, rightCP= 26, 27, 28
leftKP, rightKP= 23, 29
leftAR, leftAP= 24, 25
rightAR, rightAP= 30, 31

thefloor=p.loadSDF("court_stadium.sdf")#stadium_no_collision
p.changeDynamics(1,-1,  restitution=1)
initialPoseList= [
    0, #CHEST_Y
    0, #CHEST_P
    0, #HEAD_Y
    0, #HEAD_P

    0.2618, #L_SHOULDER_P
    -0.2618, #L_SHOULDER_R #flips -/+ by itself (originally) +
    -0.0873, #L_SHOULDER_Y
    -0.5236, #L_ELBOW_P
    -1.5708, #L_ELBOW_Y
    0, #L_WRIST_P
    1.5708, #L_WRIST_Y
    0, #L_HAND

    0.2618, #R_SHOULDER_P
    0.2618, #R_SHOULDER_R #flips -/+ by itself (originally) -
    0.0873, #R_SHOULDER_Y
    -0.5236, #R_ELBOW_P
    1.5708, #R_ELBOW_Y
    0, #R_WRIST_P
    -1.5708, #R_WRIST_Y
    0, #R_HAND

    0, #L_CROTCH_Y
    0.3491, #L_CROTCH_R
    -0.2967, #L_CROTCH_P
    0.6283, #L_KNEE_P
    -0.3491, #L_ANKLE_R
    -0.3316, #L_ANKLE_P

    0, #R_CROTCH_Y
    -0.3491, #R_CROTCH_R
    -0.2967, #R_CROTCH_P
    0.6283, #R_KNEE_P
    0.3491, #R_ANKLE_R
    -0.3316 #R_ANKLE_P
]

p.changeVisualShape(krobot, 1 , rgbaColor= [1,1,0,0.3])
p.changeVisualShape(krobot, 2 , rgbaColor= [1,0,0,0.3])
p.changeVisualShape(krobot, 3 , rgbaColor= [0,1,0,0.3])
p.changeVisualShape(krobot, 0 , rgbaColor= [1,0,1,0.4])
p.changeVisualShape(krobot, -1 , rgbaColor= [0,1,1,0.6])# p.changeVisualShape(krobot,1,rgbaColor=[0.003127, 1, 0.003127, 1],specularColor=[0,0,0]) #CHEST
p.changeVisualShape(krobot,49,rgbaColor=[1, 0.003127, 0.003127, 1]) #left foot
p.changeVisualShape(krobot,69,rgbaColor=[0.003127, 0.003127, 1, 1]) #right foot
p.changeVisualShape(krobot,11,rgbaColor=[0.2,0.2,0.2, 1]) #right elb
p.changeVisualShape(krobot,8,rgbaColor=[0.2,0.2,0.2, 1]) #right foot
p.changeVisualShape(krobot,14,rgbaColor=[0.2,0.2,0.2, 1]) #right foot
# p.changeVisualShape(krobot,32,rgbaColor=[0.2,0.2,0.2, 1]) #left thai
p.changeVisualShape(krobot,52,rgbaColor=[0.2,0.2,0.2, 1]) #right thai
p.changeVisualShape(krobot,25,rgbaColor=[0.2,0.2,0.2, 1]) #left thai
# p.changeVisualShape(krobot,61,rgbaColor=[0.2,0.2,0.2, 1]) #right thai

fiba3x3ball = p.loadURDF("/home/admin/3x3/fiba3x3.urdf",[0.4,0.3,0], useFixedBase=0, globalScaling=1)
wnbaball = p.loadURDF("/home/admin/3x3/wnba.urdf",[0,0.6,0], useFixedBase=0, globalScaling=1)
nbaball = p.loadURDF("/home/admin/3x3/nba.urdf", [0.4, 0.5, 0], useFixedBase=0, globalScaling =1)
sphere = p.loadURDF("sphere_with_restitution.urdf", [-1, 0.6 ,1], useFixedBase=0, globalScaling = 1)

totaljoints = p.getNumJoints(krobot)
jointindexlist = range(totaljoints)
# jointstateslist = p.getJointStates(krobot, jointindexlist)
# print ("number of jointz =  {}".format(totaljoints))
chestp_add = p.addUserDebugParameter("CHEST_P",-3.14,3.14,0)
lshoulderp_add = p.addUserDebugParameter("Lshoulder_P",-3.14159,3.14159,0)
lelbowp_add = p.addUserDebugParameter("left bicep",-2.26893,0,0)
rbicep_add = p.addUserDebugParameter("right bicep", -2.26893, 0, 0)

# p.setAdditionalSearchPath(pybullet_data.getDataPath())
restitutionId = p.addUserDebugParameter("restitution", 0, 1, 0.94)
restitutionVelocityThresholdId = p.addUserDebugParameter("res. vel. threshold", 0, 3, 0.2)
lateralFrictionId = p.addUserDebugParameter("lateral friction", 0, 1, 0.5)
spinningFrictionId = p.addUserDebugParameter("spinning friction", 0, 1, 0.3)
rollingFrictionId = p.addUserDebugParameter("rolling friction", 0, 1, 0.1)

whatjoint = 0
jointinfo = p.getJointInfo(krobot, whatjoint)
# head_xyz = p.getLinkState(0,3)#[0][2] #:31)#(1,0)[12]
# print ("partsXYZ:-1: ", head_xyz)
# print(f"JONTS ARE  = {jointinfo}")

ball = p.loadURDF("soccerball.urdf",[2,0,1], globalScaling=1)
p.changeDynamics(ball,-1,restitution=1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
# p.changeVisualShape(krobot,11,rgbaColor=[0.8,0.6,0,1],specularColor=[0,0,0])
# # p.changeVisualShape(krobot,-1,rgbaColor=[0,0,0,0.8],specularColor=[0,0,0]) 

# p.changeVisualShape(krobot,69,rgbaColor=[0.6,0.6,0.6,0.9], specularColor=[0,0,0]) #RIGHT FOOT
# for i in jointindexlist:
# p.setJointMotorControl2(krobot, 6, p.POSITION_CONTROL, targetPosition = lshoulderp_read)
    # p.changeVisualShape(krobot,i,rgbaColor=[0.6,0.6,0.6,1])


# boxId = pybullet.loadURDF("/home/admin/khi_ros_bridge_tutorials/models/RHP5EG.urdf",cubeStartPos, cubeStartOrientation)
# for i in range (totaljoints):
timestep = 1/500
p.setTimeStep(timestep)
simulated_velocity_list = []
physical_velocity_list = []
time = []
i = 0
# for i in jointindexlist[:20]:
#     p.changeDynamics(krobot, i, restitution=1)
#     p.resetJointState(krobot, i, np.random.uniform(low=-0.1, high=0.1) )
for i in jointindexlist:
    # p.resetJointState(krobot, i, initialPoseList[i] )

    print(p.getJointInfo(krobot,i))
    print (jointindexlist)

    #######################
# p.getContactPoints(krobot, thefloor, -1, -1)
 ###################



while p.isConnected():
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING) 

    time.append(i * timestep)
    physical_velocity_list.append( i * timestep)
    simulated_velocity_list.append(p.getBaseVelocity(wnbaball)[0][2])
    if (p.getBasePositionAndOrientation(wnbaball)[0][2] <= 0.05):
        print(i * timestep)
        break

    # chestp_read = p.readUserDebugParameter(chestp_add)
    # p.setJointMotorControl2(krobot, 1, p.POSITION_CONTROL, targetPosition = chestp_read)
    # lshoulderp_read = p.readUserDebugParameter(lshoulderp_add)
    # p.setJointMotorControl2(krobot, 7, p.POSITION_CONTROL, targetPosition = lshoulderp_read)     
    # lelbowp_read= p.readUserDebugParameter(lelbowp_add)
    # p.setJointMotorControl2(krobot, 11, p.POSITION_CONTROL, targetPosition = lelbowp_read)
    # rightbicep_read=p.readUserDebugParameter(rbicep_add)
    # p.setJointMotorControl2(krobot, 22, p.POSITION_CONTROL, targetPosition = rightbicep_read)
    restitution = p.readUserDebugParameter(restitutionId)
    restitutionVelocityThreshold = p.readUserDebugParameter(restitutionVelocityThresholdId)
    p.setPhysicsEngineParameter(restitutionVelocityThreshold=restitutionVelocityThreshold)
    lateralFriction = p.readUserDebugParameter(lateralFrictionId)
    spinningFriction = p.readUserDebugParameter(spinningFrictionId)
    rollingFriction = p.readUserDebugParameter(rollingFrictionId)
    # for i in thefloor:
    #     p.changeDynamics(i, -1, lateralFriction=1, restitution = 1)

    head_xyz = p.getLinkState(0,2)[0][2] #:31)#(1,0)[12]
    head_Y_xyz = p.getLinkState(0,3)[0][2] #:31)#(1,0)[12]
    # print ("HEAD_P_Z: ", head_xyz, "|| HEAD_Y: ", head_Y_xyz)

    # p.resetDebugVisualizerCamera(2,90,-40, p.getBasePositionAndOrientation(fiba3x3ball)[0])
    # print (smallball)
        # info = p.getJointInfo(krobot, i)
        # print (info[0],":",info[1])
    # print (jointInfo[0])

    touchSelf= False
    quadhand = set(x[0] for x in  p.getContactPoints(0, 1, 19, -1)) 
    bothFeet = set(x[0] for x in  p.getContactPoints(0, 0, 31, 25)) 
    leftFootShin = set(x[0] for x in  p.getContactPoints(0, 0, 25, 29)) #L FOOT R SHIN 
    rightFootShin = set(x[0] for x in  p.getContactPoints(0, 0, 31, 23)) #R FOOT L SHIN
    leftShinQuad = set(x[0] for x in  p.getContactPoints(0, 0, 23, 28)) #L SHIN R QUAD
    rightShinQuad = set(x[0] for x in  p.getContactPoints(0, 0, 29, 22)) #R SHIN L QUAD
    bothKnees = set(x[0] for x in  p.getContactPoints(0, 0, 29, 23)) 
    bothQuads = set(x[0] for x in  p.getContactPoints(0, 0, 22, 28)) 
    leftBicepChest = set(x[0] for x in  p.getContactPoints(0, 0, 1, 6)) 
    rightBicepChest = set(x[0] for x in  p.getContactPoints(0, 0, 1, 14)) 
    leftForearmChest = set(x[0] for x in  p.getContactPoints(0, 0, -1, 8)) #L ELBOW Y
    rightForearmChest = set(x[0] for x in  p.getContactPoints(0, 0, 1, 16)) #R ELBOW Y 
    leftKnuckleChest = set(x[0] for x in  p.getContactPoints(0, 0, -1, 10)) #L ELBOW Y
    rightKnuckleChest = set(x[0] for x in  p.getContactPoints(0, 0, 1, 18)) #R ELBOW Y 

    if (quadhand):
    #   print ("QUAD AND FLOOR")
      touchSelf= True
    if (bothFeet):
    #   print ("FEET are TOUCHING")
      touchSelf= True    
    if (bothKnees):
    #   print ("SHINS are TOUCHING")
      touchSelf= True    
    if (leftFootShin or rightFootShin):
    #   print ("FOOT & SHIN are TOUCHING")
      touchSelf= True    
    if (leftShinQuad or rightShinQuad ):
    #   print ("SHIN and QUAD are TOUCHING")
      touchSelf= True     
    if (bothQuads):
    #   print ("QUADS are TOUCHING")
      touchSelf= True
    if (leftBicepChest or rightBicepChest):
    #   print ("bicep and chest contact")
      touchSelf=True
    if (leftForearmChest or rightForearmChest):
    #   print ("forearm and chest contact")
      touchSelf=True    
    if (leftKnuckleChest or rightKnuckleChest):
    #   print ("kuckle and chest contact")
      touchSelf=True        

    # playerDown= False
    for i in range(p.getNumJoints(krobot)):
      bodyFloor= set(x[0] for x in  p.getContactPoints(1, 0, -1, i))
      leftHandFloor = set(x[0] for x in  p.getContactPoints(1, 0, -1, leftH))
      leftWristFloor = set(x[0] for x in  p.getContactPoints(1, 0, -1, leftWY))
      rightWristFloor = set(x[0] for x in  p.getContactPoints(1, 0, -1, rightWY))
      rightHandFloor = set(x[0] for x in  p.getContactPoints(1, 0, -1, rightH))
      if (leftWristFloor or rightHandFloor or leftWristFloor or rightWristFloor):
          print("hand on floor")
      elif (bodyFloor):
          print ("player Down", [i])
        # playerDown= True
    #   else :# :
    # #     # playerDown= False
    #     print ("hand(s) on floor")

    # taus= PDControllerStable(p).computePD(0, )


    p.stepSimulation()
    # time.sleep(1./500.)
    # p.resetJointState(krobot, 22, 0)
# p.disconnect()

