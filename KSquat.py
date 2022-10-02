#41 KHI SQUAT POSE

from squat_robot_bases import URDFBasedRobot#, BodyPart
import numpy as np
import pybullet, pybullet_data, os
# import rospy, actionlib
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
class WalkerBaseURDF(URDFBasedRobot):

  def __init__(self, fn, robot_name, action_dim, obs_dim, power):
    URDFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim)
    self.power = power
    # self.camera_x = 0
    # self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
    self.walk_target_x = 0.01  # kilometer away
    self.walk_target_y = 0
    # self.waistXYZ = [0, 0, 0]
    self.base_id=-1
    # self.chestY_id =1
    # self.chestP_id= 0
    # self.headY_id, self.headP_id= 2, 3
    #36 uses shoulders, elbow_P, LEGS_P_P_P
    self.LButRSP, self.leftbutRightSR, self.leftButRSY= 0, 1, 2 #SHOULDER PAD // BONE // BIandTRICEP
    self.leftEP =3
    self.rightButLSP, self.rightButLeftSR, self.rightSY= 10, 11, 12
    # self.leftEY= 8 #ELBOW PAD // FOREARM

    # self.leftWP, self.leftWY, self.leftH= 9, 10, 11 #HAND SENSOR // KNUCKLE// FINGERS
    # self.rightWP, self.rightWY, self.rightH= 17, 18, 19
    self.rightEP = 13
    # self.rightEY= 
    
    # self.leftCY =9
    # self.leftCR =10 
    self.leftCP= 7 # ABOVE QAUD // TINY BALL // THIGH 
    self.leftKP = 8
    # self.leftAR = 11
    self.leftAP= 9
    # self.rightCY =15
    # self.rightCR =16
    self.rightCP= 4
    self.rightKP= 5
    # self.rightAR = 19
    self.rightAP= 6

  def setup_collision_group(self, link_a_ids, link_b_ids, flag):
      [self.setup_collision(i, j, flag) for i in link_a_ids for j in link_b_ids]

  def setup_collision(self, link_a, link_b, flag):
      if link_a != link_b: pybullet.setCollisionFilterPair(1, 1, link_a, link_b, flag)

  def noCollision(self):
    BODY_LINK_IDS = tuple(range(1))# + (16, 17))
    # ARM4BODY_LINK_IDS = tuple((4, 17) + tuple(range(18, 27)))
    L_ARM_LINK_IDS = tuple(range(4))
    R_ARM_LINK_IDS = tuple(range(4))
    L_LEG_LINK_IDS = tuple(range(3))
    R_LEG_LINK_IDS = tuple(range(3))
    all_link_ids = tuple(range(15))    
    self.setup_collision_group(all_link_ids, all_link_ids, False)
    # self.setup_collision_group(L_ARM_LINK_IDS, BODY_LINK_IDS, True)
    # self.setup_collision_group(R_ARM_LINK_IDS, BODY_LINK_IDS, True)
    # self.setup_collision_group(R_ARM_LINK_IDS, L_ARM_LINK_IDS, True)
    # self.setup_collision_group(R_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
    # self.setup_collision_group(R_ARM_LINK_IDS, L_LEG_LINK_IDS, True)
    # self.setup_collision_group(L_ARM_LINK_IDS, L_LEG_LINK_IDS, True)    
    # self.setup_collision_group(L_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
    # self.setup_collision_group(L_LEG_LINK_IDS, R_LEG_LINK_IDS, True)
    # pybullet.setCollisionFilterPair(1, 1, self.headY_id, self.chestP_id, True    )

  def colorTF(self):

    pybullet.addUserDebugText("base base base", [0.15, -0.1, 0], textSize=1.3, parentObjectUniqueId=1, parentLinkIndex=-1, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.3,0,0], [-0.3, 0,0], [1,0,0], 8, parentObjectUniqueId=1, parentLinkIndex=-1)
    pybullet.addUserDebugLine([0,-0.4,0], [0,0.4,0], [0,2,0], 7, parentObjectUniqueId=1, parentLinkIndex=-1)
    pybullet.addUserDebugLine([0,0,-1.1], [0,0,0.15], [0,0,1], 7, parentObjectUniqueId=1, parentLinkIndex=-1)

    pybullet.addUserDebugText("HEAD_Y", [0.14, 0, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.headY_id, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.3,0,0], [-0.3, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.headY_id)
    pybullet.addUserDebugLine([0,-0.4,0], [0,0.4,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.headY_id)
    pybullet.addUserDebugLine([0,0,-1.1], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.headY_id)

    pybullet.addUserDebugText("CHEST_Y", [0.2, 0, 0], textSize=2, parentObjectUniqueId=1, parentLinkIndex=self.chestY_id, textColorRGB=[0.7,1.5,0])
    pybullet.addUserDebugLine([0.2,0,0], [-0.2,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestY_id)
    pybullet.addUserDebugLine([0,0.2,0], [0,-0.2,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestY_id)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0.15,0.2], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestY_id)

    pybullet.addUserDebugText("CHEST_P", [0.2, 0, 0], textSize=2, parentObjectUniqueId=1, parentLinkIndex=self.chestP_id, textColorRGB=[0.7,1.5,0])
    pybullet.addUserDebugLine([0.2,0,0], [-0.2,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestP_id)
    pybullet.addUserDebugLine([0,0.2,0], [0,-0.2,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestP_id)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0.15,0.2], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.chestP_id)

    pybullet.addUserDebugText("HEAD_P", [0.1, 0, 0.1], textSize=1.5, parentObjectUniqueId=1, parentLinkIndex=self.headP_id, textColorRGB=[0.7,1.5,0])
    pybullet.addUserDebugLine([0.2,0,0], [-0.2,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.headP_id)
    pybullet.addUserDebugLine([0,0.2,0], [0,-0.2,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=3)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.2], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=3)

    pybullet.addUserDebugText("(LEFT)R_SR", [0.05, -0.17, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftbutRightSR, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftbutRightSR)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftbutRightSR)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftbutRightSR)

    pybullet.addUserDebugText("(LEFT)R_SP", [0.05, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.LButRSP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.17,0,0], [-0.17, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.LButRSP)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.LButRSP)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.LButRSP)

    pybullet.addUserDebugText("(LEFT)R_SY", [0, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftButRSY, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftButRSY)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftButRSY)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftButRSY)   
    
    pybullet.addUserDebugText("(LEFT)R_EP", [0, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftEP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEP)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEP)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEP)
    
    pybullet.addUserDebugText("(LEFT)R_EY", [0, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftEY, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEY)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEY)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftEY)

    pybullet.addUserDebugText("(LEFT)R_WP", [0, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftWP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWP)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWP)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWP)    

    pybullet.addUserDebugText("(LEFT)R_WY", [0, -0.2, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftWY, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWY)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWY)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftWY) 

    pybullet.addUserDebugText("(RIGHT)L_SP", [0, 0.12, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightButLSP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.17,0,0], [-0.17, 0,0], [2,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLSP)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLSP)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLSP)

    pybullet.addUserDebugText("(RIGHT)L_SR", [0, 0.12, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightButLeftSR, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.17,0,0], [-0.17, 0,0], [2,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLeftSR)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLeftSR)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightButLeftSR)

    pybullet.addUserDebugText("(RIGHT)L_EP", [0, 0.15, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightEP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1, 0, 0], [-0.1, 0, 0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEP)
    pybullet.addUserDebugLine([0, -0.1, 0], [0, 0.1, 0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEP)
    pybullet.addUserDebugLine([0, 0, -0.1], [0, 0, 0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEP)

    pybullet.addUserDebugText("(RIGHT)L_EY", [0, 0.15, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightEY, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1, 0, 0], [-0.1, 0, 0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEY)
    pybullet.addUserDebugLine([0, -0.1, 0], [0, 0.1, 0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEY)
    pybullet.addUserDebugLine([0, 0, -0.1], [0, 0, 0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightEY)

    pybullet.addUserDebugText("(RIGHT)L_WP", [0, 0.17, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightWP, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1,0,0], [-0.1, 0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWP)
    pybullet.addUserDebugLine([0,-0.1,0], [0,0.1,0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWP)
    pybullet.addUserDebugLine([0,0,-0.1], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWP)
  
    pybullet.addUserDebugText("(RIGHT)L_WY", [0, 0.15, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightWY, textColorRGB=[0.4,1.6,0])
    pybullet.addUserDebugLine([0.1, 0, 0], [-0.1, 0, 0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWY)
    pybullet.addUserDebugLine([0, -0.1, 0], [0, 0.1, 0], [0,2,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWY)
    pybullet.addUserDebugLine([0, 0, -0.1], [0, 0, 0.1], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightWY)

    pybullet.addUserDebugText("L_CROTCH_P", [0.1, 0.1, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftCP, textColorRGB=[0,3,0])
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCP)
    pybullet.addUserDebugLine([0,0.15,0], [0,-0.1,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCP)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCP)

    pybullet.addUserDebugText("L_CROTCH_Y", [0.1, 0.1, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftCY, textColorRGB=[0,3,0])
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCY)
    pybullet.addUserDebugLine([0,0.15,0], [0,-0.1,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCY)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftCY)

    pybullet.addUserDebugText("LEFT_KNEE", [0, -0.17, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightKP, textColorRGB=[0,3,0])
    pybullet.addUserDebugLine([0.18,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightKP)
    pybullet.addUserDebugLine([0,0.09,0], [0,-0.15,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightKP)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightKP)
   
    pybullet.addUserDebugText("L_ANKLE_R", [0, 0.1, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftAR, textColorRGB=[1,3,0])
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAR)
    pybullet.addUserDebugLine([0,0.15,0], [0,-0.09,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAR)
    pybullet.addUserDebugLine([0,0,-0.2], [0,0,0.2], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAR)

    pybullet.addUserDebugText("L_ANKLE_P", [0.1, 0.1, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftAP, textColorRGB=[1,3,0])
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAP)
    pybullet.addUserDebugLine([0,0.15,0], [0,-0.09,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAP)
    pybullet.addUserDebugLine([0,0,-0.2], [0,0,0.2], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftAP)

    pybullet.addUserDebugText("R_CROTCH_P", [0.1, -0.12, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightCP, textColorRGB=[0,3,0])
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightCP)
    pybullet.addUserDebugLine([0, -0.15,0], [0,0.1,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightCP)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.rightCP)
    
    pybullet.addUserDebugText("RIGHT_KNEE", [0.1, 0.1, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.leftKP, textColorRGB=[0,1.3,0])
    pybullet.addUserDebugLine([0.18,0,0], [-0.15,0,0], [1,0,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftKP)
    pybullet.addUserDebugLine([0,-0.09,0], [0,0.15,0], [0,1,0], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftKP)
    pybullet.addUserDebugLine([0,0,-0.15], [0,0,0.15], [0,0,1], 3, parentObjectUniqueId=1, parentLinkIndex=self.leftKP) 

    pybullet.addUserDebugText("R_ANKLE_P", [0.1, -0.18, 0], textSize=1, parentObjectUniqueId=1, parentLinkIndex=self.rightAP, textColorRGB=[0,3,0])    
    pybullet.addUserDebugLine([0.15,0,0], [-0.15,0,0], [1,0,0], 4, parentObjectUniqueId=1, parentLinkIndex=self.rightAP)
    pybullet.addUserDebugLine([0,-0.15,0], [0,0.09,0], [0,1,0], 4, parentObjectUniqueId=1, parentLinkIndex=self.rightAP)    
    pybullet.addUserDebugLine([0,0,-0.2], [0,0,0.2], [0,0,1], 4, parentObjectUniqueId=1, parentLinkIndex=self.rightAP)

    # pybullet.addUserDebugText("LEFT SHOULDER", [0.1, 0.15, 0.05], textSize=1, parentObjectUniqueId=1, parentLinkIndex=12, textColorRGB=[1,1,0])
    # pybullet.addUserDebugText("RIGHT SHOULDER", [0.1, -0.5, 0.05], textSize=1, parentObjectUniqueId=1, parentLinkIndex=4, textColorRGB=[1,1,0])
    # pybullet.addUserDebugText("R_KNEE", [-0.0, -0.35, 0], textSize=2, parentObjectUniqueId=1, parentLinkIndex=29, textColorRGB=[1,0.3,0])
    # pybullet.addUserDebugText("L_KNEE", [-0.0, 0.1, 0], textSize=2, parentObjectUniqueId=1, parentLinkIndex=23, textColorRGB=[1,0.3,0])

    pybullet.changeVisualShape(1, self.rightCP , rgbaColor= [0.2,0.6, 1,1])
    pybullet.changeVisualShape(1, self.leftCY, rgbaColor= [0,1, 0,1])
    pybullet.changeVisualShape(1, self.leftCP , rgbaColor= [1,0,0,0.7])
    pybullet.changeVisualShape(1, self.leftEY, rgbaColor= [1, 0.2, 0.7, 1])
    pybullet.changeVisualShape(1, self.rightEY, rgbaColor= [0.29, 0.29, 0.28, 1])
    pybullet.changeVisualShape(1, self.rightEP, rgbaColor= [0.28, 0.28, 0.28, 1])
    pybullet.changeVisualShape(1, self.leftButRSY, rgbaColor= [0.2, 0.2, 0.2, 1])

  def printTime(self):
    self.t+=self.dt
    # txtid = pybullet.addUserDebugText("time="+str(self.t), [0,0,2],replaceItemUniqueId=self.timeid)
    print ("time = ", self.t)
  
  def myResetFunc(self):
    # self.colorTF()
    floorFriction= 1
    pybullet.changeDynamics(0, -1, mass=0 
                            ,lateralFriction= floorFriction, rollingFriction=floorFriction, spinningFriction=floorFriction 
                            ,linearDamping=1, angularDamping=1 
                            ,contactDamping=-1 ,contactStiffness=-1
                            ,restitution= 0
                            ,frictionAnchor=1)
    pybullet.enableJointForceTorqueSensor(1, self.leftAP, enableSensor= 1)
    pybullet.enableJointForceTorqueSensor(1, self.rightAP, enableSensor= 1)

  def robot_specific_reset(self, bullet_client):
    pybullet = bullet_client
    self.t = 0; self.no_of_steps= 0

    KHI_Walk_poseList_14= [
      0.7189, #0 R_SHOULDER_P pybullet LEFT
      -0.4477, #1 R_SHOULDER_R pyBULLET LEFT
      -0.0372, #2 R_SHOULDER_Y pyBULLET LEFT
      -1.6846, #3 R_ELBOW_P
      
      -0.8428, #4 R_CROTCH_P
      1.6008, #5 R_KNEE_P
      -0.7579, #6 R_ANKLE_P      
      
      -0.8428, #7 L_CROTCH_P
      1.6008, #8 L_KNEE_P
      -0.7579, #9 L_ANKLE_P

      0.7189,  #10 L_SHOULDER_P pybullet RIGHT
      0.4477, #11 L_SHOULDER_R choreonoid negative
      0.0372, #12 L_SHOULDER_Y
      -1.6846 #13 L_ELBOW_P
    ]
    # print (self.ordered_joints)
    for i, j in zip(range(0,14),self.ordered_joints): #zip range because joint type not iterable
    # for i in range(pybullet.getNumJoints(1)):
      lo, hi= pybullet.getJointInfo(1,i)[8], pybullet.getJointInfo(1, i)[9]
      footFriction=1; footDamping=1; damping= 3
      if i==self.leftAP or i==self.rightAP: pybullet.changeDynamics(1, i
                                            ,lateralFriction=footFriction, rollingFriction=footFriction, spinningFriction=footFriction
                                            ,linearDamping=footDamping, angularDamping=footDamping
                                            ,contactDamping=-1 ,contactStiffness=-1 
                                            ,restitution=0
                                            ,frictionAnchor=1 ,contactProcessingThreshold=1
                                            )
      else: pybullet.changeDynamics(1, i, lateralFriction=1, rollingFriction=1, spinningFriction=1,
                                    linearDamping=1, angularDamping=1, jointDamping=1, restitution=0)
      j.disable_motor()
      # j.reset_current_position(0, 0)
      j.reset_current_position(KHI_Walk_poseList_14[i], 0)

    self.initWaistH= pybullet.getBasePositionAndOrientation(1)[0][2]  
    self.initWaistXYZ= pybullet.getBasePositionAndOrientation(1)[0]
    # self.initLKnee= pybullet.getLinkState(1, self.leftKP)[0][2]
    # self.initRKnee= pybullet.getLinkState(1, self.rightKP)[0][2]

    # self.feet = [self.link_dict[f] for f in self.foot_list]
    # self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
    self.feet_ground_contact()
    # self.feet_on_ground=np.array([0 for i in self.foot_list], dtype=np.float32) 
    self.scene.actor_introduce(self)
    
  def basketball(self):
    # fiba3x3= pybullet.loadURDF("/home/admin/3x3/fiba3x3.urdf",[-0.4,0.3,1.5])
    # wnbaball = pybullet.loadURDF("/home/admin/3x3/wnba.urdf",[-0.4,-0.2,1], useFixedBase=0, globalScaling=1)
    nbaball = pybullet.loadURDF("/home/admin/3x3/nba.urdf", [0.2, 0.170, 0.01], useFixedBase=0)
    # sphere = pybullet.loadURDF("sphere_with_restitution.urdf", [-1, 0.6 ,3], useFixedBase=0, globalScaling = 1)
    # pybullet.changeDynamics(sphere, -1, restitution=2 )
    pybullet.changeDynamics(nbaball, -1, restitution=1 )
    # pybullet.changeDynamics(wnbaball, -1, restitution=1 )
    # pybullet.changeDynamics(fiba3x3, -1, restitution=1.2 )
    pybullet.addUserDebugText("NBA BALL", [0, 0,0.2], textSize=0.6, parentObjectUniqueId=nbaball, parentLinkIndex=-1, textColorRGB=[1,1,0])
    # pybullet.addUserDebugText("WNBA BALL", [0, 0,0.25], textSize=0.6, parentObjectUniqueId=wnbaball, parentLinkIndex=-1, textColorRGB=[1,1,0])    
    # pybullet.addUserDebugText("FIBA 3x3", [0, 0,0.25], textSize=0.6, parentObjectUniqueId=fiba3x3, parentLinkIndex=-1, textColorRGB=[1,1,0])    
    # return pybullet.getBasePositionAndOrientation(nbaball)[0][2]

  def calc_potential(self):
    # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
    # all rewards have rew/frame units and close to 1.0
    debugmode = 0
    if (debugmode):
      # print("--- from KStand.py ---")
      print("calc_potential: self.walk_target_dist", self.walk_target_dist)
      print("self.scene.dt: ", self.scene.dt)
      print("self.scene.frame_skip: ", self.scene.frame_skip)
      print("self.scene.timestep: ", self.scene.timestep)
    return -self.walk_target_dist / self.scene.dt

  def self_collision_function(self):
    touchSelf= False
    # quadhand = set(x[0] for x in  pybullet.getContactPoints(1, 1, 6, 1)) 
    bothFeet = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftAR, self.rightAR)) 
    leftFootShin = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftAP, self.rightKP)) #L FOOT R SHIN 
    rightFootShin = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.rightAR, self.leftKP)) #R FOOT L SHIN
    leftShinQuad = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftKP, self.rightCP)) #L SHIN R QUAD
    rightShinQuad = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.rightKP, self.leftCP)) #R SHIN L QUAD
    bothKnees = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftKP, self.rightKP)) 
    bothQuads = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftCP, self.rightCP)) 
    leftBicepChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.leftButRSY)) 
    rightBicepChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.rightSY)) 
    leftForearmChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.leftEY)) #L ELBOW Y
    rightForearmChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.rightEY)) #R ELBOW Y 
    leftKnuckleChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.leftWY)) #L ELBOW Y
    rightKnuckleChest = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.chestP_id, self.rightWY)) #R ELBOW Y 
    rightKnuckleWaist = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.base_id, self.rightWY)) #R ELBOW Y 
    leftKnuckleWaist = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.base_id, self.leftWY)) #R ELBOW Y 
    rightForearmWaist = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.base_id, self.rightEY)) #R ELBOW Y 
    leftForearmWaist = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.base_id, self.leftEY))
    rightKnuckleQuad = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.rightCP, self.rightWY)) #R ELBOW Y 
    leftKnuckleQuad = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.leftCP, self.leftWY)) #R ELBOW Y    
    headLeftShoulder = set(x[0] for x in  pybullet.getContactPoints(1, 1, self.headP_id, self.rightButLSP))

    # if (quadhand):
    #   # print ("QUAD AND HAND")
    #   self.touchSelf= True
    if (headLeftShoulder):
      # print ("HEAD and shoulders shjampoo")
      touchSelf= True
    if (bothFeet):
      # print ("FEET are TOUCHING")
      touchSelf= True    
    if (bothKnees):
      # print ("SHINS are TOUCHING")
      touchSelf= True    
    if (leftFootShin or rightFootShin):
      # print ("FOOT & SHIN are TOUCHING")
      touchSelf= True    
    if (leftShinQuad or rightShinQuad ):
      # print ("SHIN and QUAD are TOUCHING")
      touchSelf= True     
    if (bothQuads):
      # print ("QUADS are TOUCHING")
      touchSelf= True
    if (leftBicepChest or rightBicepChest):
      # print ("bicep and chest contact")
      touchSelf=True
    if (leftForearmChest or rightForearmChest):
      # print ("forearm and chest contact")
      touchSelf=True    
    if (leftKnuckleChest or rightKnuckleChest):
      # print ("kuckle and chest contact")
      touchSelf=True        
    if (leftKnuckleWaist or rightKnuckleWaist):
      # print ("kuckle and WAIST contact")
      touchSelf=True     
    if (leftKnuckleQuad or rightKnuckleQuad):
      # print ("kuckle and QUAD contact")
      touchSelf=True     
    if (leftForearmWaist or rightForearmWaist):
      # print ("FOREARM & WAIST")
      touchSelf=True    
    return -0.75 if touchSelf is True else 0

  def waistEulerAlive(self):
    reward =10
    rollDeg, pitchDeg = np.rad2deg(self.waistAngleEuler[0]), np.rad2deg(self.waistAngleEuler[1])
    waistDrop = self.initWaistH - self.waistXYZ[2]
    # print ("PITCH: ", format(pitchDeg, ".2f"), "H: ", format(waistDrop, ".2f"), "ROLL: ", format(rollDeg, ".2f"))
    if abs(pitchDeg) > 30 or waistDrop > 0.10: reward = -3
    if abs(pitchDeg) <=30 and abs(pitchDeg) > 25: reward =0
    if abs(pitchDeg) <=25 and abs(pitchDeg) > 20: reward =0.25
    if abs(pitchDeg) <=20 and abs(pitchDeg) > 15: reward =0.5
    if abs(pitchDeg) <=15 and abs(pitchDeg) > 10: reward =1
    if abs(pitchDeg) <=10 and abs(pitchDeg) >  5: reward =2
    if abs(pitchDeg) <= 5: reward =3
    return reward
    # return -3 if abs(rollDeg) > 15 or abs(pitchDeg) > 30 or waistDrop > 0.15 else +3 

  def feetUnderAlive(self):
    waistDrop = self.initWaistH - self.waistXYZ[2]
    # print ("waist has DROPPED", format(waistDrop,".2f"))
    # waistDrop = self.initHeadH - self.headP_XYZ[2]

    # pybullet.addUserDebugText(str(self.t), [0,-0.8,1.8], textSize=3)

    waistBehindAheadFeet = False
    midFeetX = (self.LAP_state[0][0] + self.RAP_state[0][0])/2
    feetXwaistX = midFeetX - self.waistXYZ[0]
    frontFoot = max(self.LAP_state[0][0], self.RAP_state[0][0])
    backFoot = min(self.LAP_state[0][0], self.RAP_state[0][0])
    # if self.waistXYZ[0] < (backFoot-0.2) or 
    if self.waistXYZ[0] > (frontFoot+0.15) or self.waistXYZ[0] < (backFoot - 0.15): waistBehindAheadFeet=True
    # if abs(feetXwaistX) > 0.26: waistBehindAheadFeet=True   
    # print ("LEFT foot X", format(self.LAP_state[0][0],".2f"), "RIGHT foot X", format(self.RAP_state[0][0],".2f"), "WAIST X", format(self.waistXYZ[0],".2f"))

    waistOutsideFoot = False
    # leftFootWaistY = self.LAP_state[0][1] - self.waistXYZ[1]
    # rightFootWaistY = self.RAP_state[0][1] - self.waistXYZ[1]
    # if leftFootWaistY<0 or rightFootWaistY>0: waistOutsideFoot= True
    if self.waistXYZ[1] > self.LAP_state[0][1] or self.waistXYZ[1] < self.RAP_state[0][1]: waistOutsideFoot= True
    # if leftFootWaistY<0: print ()
    # self.waistXYZ[1] < self.RAP_state[0][1]: waistLeanCOndition=True
    # print ("current LEFT foot Y", format(self.LAP_state[0][1],".2f"), "current WAIST Y", format(self.waistXYZ[1],".2f"))

    # waistLean = (leftFootWaistY+rightFootWaistY)/2

    debugmode=0
    if (debugmode):
      # if leftFootWaistY<0: print("waist outside LEFT") ##for X
      # if rightFootWaistY>0: print("waist outside RIGHT")
      print ("Waist X feet X", format(feetXwaistX,".2f"))
      # if feetXwaistX > 0.3: print("waist TOO far BEHIND 30cm") ##for Y
      # if feetXwaistX < 0.3: print("waist TOO far AHEAD 30cm")           
      # print ("average feet Y LEAN", format(waistLean,".2f"))
      # print ("average feet X StR8", format(waistComeAndGo,".2f"))
      # if waistComeAndGo<0: print ("feet BEHIND WAIST")
      # else: print ("feet IN FRONT OF waist")
      # if waistLean<0.3: print ("waist OUTSIDE RIGHT FOOT")
      # elif waistLean<-0.3: print ("waist OUTSIDE LEFT FOOT")
    return -3 if waistDrop > 0.15 or waistBehindAheadFeet==True else +3# or waistOutsideFoot==True else +1 

  def feet_ground_contact(self):
    rightFootFloor = set(x[0] for x in  pybullet.getContactPoints(1, 0, self.rightAP, -1)) # L_ANKLE_P_LINK
    leftFootFLoor = set(x[0] for x in  pybullet.getContactPoints(1, 0, self.leftAP, -1)) # R_ANKLE_P_LINK 
    leftFootOnGround, rightFootOnGround = 0.0, 0.0
    if (leftFootFLoor): leftFootOnGround = 1 #; print ("LEFT foot on floor:")#, self.foot_on_ground)      
    if (rightFootFloor): rightFootOnGround= 1 #; print ("RIGHT foot on floor:")#, self.foot_on_ground)
    self.feet_on_ground= np.array([leftFootOnGround, rightFootOnGround])
    return -1 if sum(self.feet_on_ground) ==0 else sum(self.feet_on_ground)

  def longLife(self, LLMultiplier):
    liveLong = LLMultiplier * (self.no_of_steps / 1500)    
    # np.clip(liveLong, 0, 2.5)
    # longLife = np.tanh(liveLong)
    return np.clip(liveLong, 0, 2.5)

  def alive_bonus(self, alive_z):#, pitch):
    # print('alive bonus Z = ', alive_z)
    # return +2 if alive_z > 0.9 else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying
    return +2 if alive_z >= LCP_state[0][2] else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying

  def calc_state(self):
    self.no_of_steps += 1
    # self.t+=1/400
    # print ("time in calc_state = ", self.t)    
    jointPV = np.array([j.current_relative_position() for j in self.ordered_joints]).flatten()#, dtype=np.float32).flatten()
    jointP =jointPV[0::2] #JOINT POSITION
    self.joint_speeds =jointPV[1::2] #velList
    jointV = self.joint_speeds
    self.joints_at_limit = np.count_nonzero(np.abs(jointP) > 0.99)
    # print(j[0::2])
    
    waistState= pybullet.getBasePositionAndOrientation(1); self.waistXYZ = waistState[0]; waist_quat = waistState[1]; self.waistAngleEuler = pybullet.getEulerFromQuaternion(waist_quat)    
    # self.chestY_state= pybullet.getLinkState(1,self.chestY_id); self.chestY_XYZ= self.chestY_state[0]; chestY_quat= self.chestY_state[1]
    # self.chestP_state= pybullet.getLinkState(1,self.chestP_id); self.chestP_XYZ= self.chestP_state[0]; chestP_quat= self.chestP_state[1]
    # self.headY_state= pybullet.getLinkState(1,self.headY_id);   self.headY_XYZ= self.headY_state[0]; headY_quat= self.headY_state[1]
    # self.headP_state= pybullet.getLinkState(1,self.headP_id);   self.headP_XYZ= self.headP_state[0]; headP_quat= self.headP_state[1]

    LSP_state= pybullet.getLinkState(1,self.LButRSP); LSP_XYZ= LSP_state[0]; LSP_quat= LSP_state[1]
    LSR_state= pybullet.getLinkState(1,self.rightButLeftSR); LSR_XYZ= LSR_state[0]; LSR_quat= LSR_state[1]
    LSY_state= pybullet.getLinkState(1,self.leftButRSY); LSY_XYZ= LSY_state[0]; LSY_quat= LSY_state[1]
    LEP_state= pybullet.getLinkState(1,self.leftEP); LEP_XYZ= LEP_state[0]; LEP_quat= LEP_state[1]
    # self.LEY_state= pybullet.getLinkState(1,self.leftEY); self.LEY_XYZ= self.LEY_state[0]; LEY_quat= self.LEY_state[1]
    # self.LWP_state= pybullet.getLinkState(1,self.leftWP); self.LWP_XYZ= self.LWP_state[0]; LWP_quat= self.LWP_state[1]
    # self.LWY_state= pybullet.getLinkState(1,self.leftWY); self.LWY_XYZ= self.LWY_state[0]; LWY_quat= self.LWY_state[1]
    # self.LH_state= pybullet.getLinkState(1,self.leftH); self.LH_XYZ= self.LH_state[0]; LH_quat= self.LH_state[1]

    RSP_state= pybullet.getLinkState(1,self.rightButLSP); RSP_XYZ= RSP_state[0]; RSP_quat= RSP_state[1]
    RSR_state= pybullet.getLinkState(1,self.leftbutRightSR); RSR_XYZ= RSR_state[0]; RSR_quat= RSR_state[1]
    RSY_state= pybullet.getLinkState(1,self.rightSY); RSY_XYZ= RSY_state[0]; RSY_quat= RSY_state[1]
    REP_state= pybullet.getLinkState(1,self.rightEP); REP_XYZ= REP_state[0]; REP_quat= REP_state[1]
    # self.REY_state= pybullet.getLinkState(1,self.rightEY); self.REY_XYZ= self.REY_state[0]; REY_quat= self.REY_state[1]
    # self.RWP_state= pybullet.getLinkState(1,self.rightWP); self.RWP_XYZ= self.RWP_state[0]; RWP_quat= self.RWP_state[1]
    # self.RWY_state= pybullet.getLinkState(1,self.rightWY); self.RWY_XYZ= self.RWY_state[0]; RWY_quat= self.RWY_state[1]
    # self.RH_state= pybullet.getLinkState(1,self.rightH); self.RH_XYZ= self.RH_state[0]; RH_quat= self.RH_state[1]

    # self.LCY_state= pybullet.getLinkState(1,self.leftCY); self.LCY_XYZ= self.LCY_state[0]; LCY_quat= self.LCY_state[1]
    # self.LCR_state= pybullet.getLinkState(1,self.leftCR); self.LCR_XYZ= self.LCR_state[0]; LCR_quat= self.LCR_state[1]
    LCP_state= pybullet.getLinkState(1,self.leftCP); LCP_XYZ= LCP_state[0]; LCP_quat= LCP_state[1]
    LKP_state= pybullet.getLinkState(1,self.leftKP); LKP_XYZ= LKP_state[0]; LKP_quat= LKP_state[1]
    # self.LAR_state= pybullet.getLinkState(1,self.leftAR); self.LAR_XYZ= self.LAR_state[0]; LAR_quat= self.LAR_state[1]
    self.LAP_state= pybullet.getLinkState(1,self.leftAP,1); LAP_XYZ= self.LAP_state[0]; LAP_quat = self.LAP_state[1]; self.LAP_rad= pybullet.getEulerFromQuaternion(LAP_quat)

    # self.RCY_state= pybullet.getLinkState(1,self.rightCY); self.RCY_XYZ= self.RCY_state[0]; RCY_quat= self.RCY_state[1]
    # self.RCR_state= pybullet.getLinkState(1,self.rightCR); self.RCR_XYZ= self.RCR_state[0]; RCR_quat= self.RCR_state[1]
    RCP_state= pybullet.getLinkState(1,self.rightCP); RCP_XYZ= RCP_state[0]; RCP_quat= RCP_state[1]
    RKP_state= pybullet.getLinkState(1,self.rightKP); RKP_XYZ= RKP_state[0]; RKP_quat= RKP_state[1]
    # self.RAR_state= pybullet.getLinkState(1,self.rightAR); self.RAR_XYZ= self.RAR_state[0]; RAR_quat= self.RAR_state[1]
    self.RAP_state= pybullet.getLinkState(1,self.rightAP,1); RAP_XYZ= self.RAP_state[0]; RAP_quat = self.RAP_state[1]; self.RAP_rad= pybullet.getEulerFromQuaternion(RAP_quat)
    LAP_sensor= pybullet.getJointState(1, self.leftAP)[2]; RAP_Sensor= pybullet.getJointState(1, self.rightAP)[2]

    # self.LAP_linearV = self.LAP_state[6]; self.RAP_linearV = self.RAP_state[6]

    roll, pitch, yaw = self.waistAngleEuler
    self.walk_target_theta = np.arctan2(self.walk_target_y - self.waistXYZ[1], self.walk_target_x - self.waistXYZ[0])
    self.walk_target_dist = np.linalg.norm([self.walk_target_y - self.waistXYZ[1], self.walk_target_x - self.waistXYZ[0]])
    angle_to_target = self.walk_target_theta - yaw
    rot_speed = np.array([[np.cos(-yaw), -np.sin(-yaw), 0], [np.sin(-yaw), np.cos(-yaw), 0], [0, 0, 1]])
    vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view
  
    WAIST_ORIENT =  waist_quat[0] ,waist_quat[1], waist_quat[2]
    LSP_orient = LSP_quat[0], LSP_quat[1], LSP_quat[2]; RSP_orient = RSP_quat[0], RSP_quat[1], RSP_quat[2]
    LEP_orient = LEP_quat[0], LEP_quat[1], LEP_quat[2]; REP_orient = REP_quat[0], REP_quat[1], REP_quat[2]
    LAP_orient = LAP_quat[0], LAP_quat[1] ,LAP_quat[2]; RAP_orient = RAP_quat[0], RAP_quat[1] ,RAP_quat[2]      
    LKP_orient = LKP_quat[0], LKP_quat[1], LKP_quat[2]; RKP_orient = RKP_quat[0], RKP_quat[1], RKP_quat[2]
    LCP_orient = LCP_quat[0], LCP_quat[1], LCP_quat[2]; RCP_orient = RCP_quat[0], RCP_quat[1], RCP_quat[2]

    LSP_XYZ_rel = np.subtract(LSP_XYZ, self.waistXYZ)
    LSR_XYZ_rel = np.subtract(LSR_XYZ, self.waistXYZ)
    LSY_XYZ_rel = np.subtract(LSY_XYZ, self.waistXYZ)
    LEP_XYZ_rel = np.subtract(LEP_XYZ, self.waistXYZ)
    RSP_XYZ_rel = np.subtract(RSP_XYZ, self.waistXYZ)
    RSR_XYZ_rel = np.subtract(RSR_XYZ, self.waistXYZ)
    RSY_XYZ_rel = np.subtract(RSY_XYZ, self.waistXYZ)    
    REP_XYZ_rel = np.subtract(REP_XYZ, self.waistXYZ)

    RCP_XYZ_rel = np.subtract(RCP_XYZ, self.waistXYZ)
    RKP_XYZ_rel = np.subtract(RKP_XYZ, self.waistXYZ)
    RAP_XYZ_rel = np.subtract(RAP_XYZ, self.waistXYZ)
    LCP_XYZ_rel = np.subtract(LCP_XYZ, self.waistXYZ)
    LKP_XYZ_rel = np.subtract(LKP_XYZ, self.waistXYZ)
    LAP_XYZ_rel = np.subtract(LAP_XYZ, self.waistXYZ)

    LAP_force = LAP_sensor[0], LAP_sensor[1] ,LAP_sensor[2]; LAP_MoM = LAP_sensor[3], LAP_sensor[4] ,LAP_sensor[5]
    RAP_force = RAP_Sensor[0], RAP_Sensor[1] ,RAP_Sensor[2]; RAP_MoM = RAP_Sensor[3], RAP_Sensor[4] ,RAP_Sensor[5]
    L_ARM = [LSP_XYZ_rel] + [LEP_XYZ_rel]; R_ARM = RSP_XYZ_rel + REP_XYZ_rel
    L_LEG_rEL = LCP_XYZ_rel + LKP_XYZ_rel + LAP_XYZ_rel; R_LEG_ReL = RCP_XYZ_rel + RKP_XYZ_rel + RAP_XYZ_rel
    
    # print ("jay for ",1,1000*j[1])
    # print ("self.no_of_steps: ", self.no_of_steps)
    # print ("J", [j])
    # return np.array(posList) #, velList)

    # print ("wQuat", np.tanh(([LSP_XYZ_tuple] - [LEP_XYZ_tuple])))
    # print ("LSP XYZ", np.tanh(np.subtract(LSP_XYZ_tuple , self.waistXYZ)))    
    # print ("LSP XYZ tanh", (np.subtract(LSP_XYZ_tuple , self.waistXYZ)))
    # print ("W raw lap f",np.divide(LAP_MoM,1000))
    # print ("W tan LAP mom",np.tanh (np.divide(LAP_MoM,1000)))
    # print ("W tan RAP mom",np.tanh (np.divide(RAP_MoM,1000)))
    # print (type(L_ARM))
    # print ("[]:", [L_ARM])
    # print ("LAP rel XYZ",LAP_XYZ_rel)
    # print ("LAP tanh XYZ",np.tanh(LAP_XYZ_rel))
    # print ("LSR rel XYZ",LSR_XYZ_rel)
    # print ("LSY rel XYZ",LSY_XYZ_rel)
    # print ("wOrient", WAIST_ORIENT)

    return np.concatenate(
            [jointPV] #28
          + [self.waistXYZ] 
          # + [WAIST_ORIENT] #6

          + [LSP_XYZ_rel] + [LEP_XYZ_rel]  #left arm XYZ
          + [RSP_XYZ_rel] + [REP_XYZ_rel]  #right arm XYZ

          + [LCP_XYZ_rel] + [LKP_XYZ_rel] 
          + [LAP_XYZ_rel] 
          + [RCP_XYZ_rel] + [RKP_XYZ_rel] 
          + [RAP_XYZ_rel]  #12 + 18 = 30

          # + [LSP_orient] + [LEP_orient]
          # + [RSP_orient] + [REP_orient] #ARM ORIENT = 12

          # + [LCP_orient] + [RCP_orient] #Crotch RPY
          # + [LKP_orient] + [RKP_orient] #Knee RPY
          # + [LAP_orient] + [RAP_orient] #FEET ORIENT = 6

          + [np.tanh(np.divide(LAP_force,1000))] + [np.tanh(np.divide(RAP_force,1000))]
          # + [np.tanh(np.divide(LAP_MoM,1000))] + [np.tanh(np.divide(RAP_MoM,1000))] #6
          # + [self.feet_on_ground] 
          )
    # return np.clip(np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

class KSquatClass(WalkerBaseURDF):
  # self_collision = False #works without so ...
  foot_list = ["R_ANKLE_P_LINK", "L_ANKLE_P_LINK"]
  def __init__(self): WalkerBaseURDF.__init__(self,
    '/home/admin/dribble_repo/RHP5E_14.urdf', #mod2.urdf',
    # '/home/admin/dribble_repo/RHP5E_gundam.urdf',
    # '/home/admin/dribble_repo/RHP5E_chappie.urdf',
    # '/home/jovyan/private/drb/urdfs/RHP5E_14.urdf',
    'robot_name_WBURDF',
    action_dim= 14,  #no of joints to play with
    obs_dim= 58, 
    power= 0.3) #because 30% EFFORT ????
    # 17 joints, 4 of them important for walking (hip, knee), others may as well be turned off, 17/4 = 4.25
    # 32 / 13 important standing joints = 2.461538462

  def robot_specific_reset(self, bullet_client):
    WalkerBaseURDF.robot_specific_reset(self, bullet_client)
    # self.motor_names  = ["CHEST_Y"];  self.motor_power = [347.5636]; self.pGainList = [44000]; self.vGainList = [400]; self.maxVelList = [366.667]
    # self.motor_names = ["CHEST_P"]; self.motor_power = [347.5636]; self.pGainList= [44000]; self.vGainList= [440]; self.maxVelList= [366.667]
    # self.motor_names += ["HEAD_Y"];  self.motor_power += [22.2]; self.pGainList+= [2000]; self.vGainList+= [50]; self.maxVelList+= [1800] 
    # self.motor_names += ["HEAD_P"];  self.motor_power += [22.2]; self.pGainList+= [2000]; self.vGainList+= [50]; self.maxVelList+= [1800]
    # self.motor_names += ["LC-W-base-joint", "LC-W-linear-joint"]; self.motor_power += [0.0, 0.0]    
    self.motor_names = ["L_SHOULDER_P"]; self.motor_power = [178.42]; self.pGainList= [15000]; self.vGainList= [240]; self.maxVelList= [357.143]  
    self.motor_names += ["L_SHOULDER_R"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714]
    self.motor_names += ["L_SHOULDER_Y"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714] 
    # self.motor_names += ["LC-E-base-joint", "LC-E-linear-joint"]; self.motor_power += [0.0, 0.0]
    self.motor_names += ["L_ELBOW_P"]; self.motor_power += [247.16]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.813]
    # self.motor_names += ["L_ELBOW_Y"]; self.motor_power += [123.62]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.727]    
    # self.motor_names += ["L_WRIST_P"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400] 
    # self.motor_names += ["L_WRIST_Y"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]  
    # self.motor_names += ["L_HAND"];    self.motor_power += [94.4];   self.pGainList+= [1];     self.vGainList+= [0.1]; self.maxVelList+= [675]       
    # self.motor_names += ["RC-W-base-joint", "RC-W-linear-joint"]; self.motor_power += [0.0, 0.0]
    self.motor_names += ["R_SHOULDER_P"]; self.motor_power += [178.42]; self.pGainList+= [15000]; self.vGainList+= [240]; self.maxVelList+= [357.143]  
    self.motor_names += ["R_SHOULDER_R"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714]
    self.motor_names += ["R_SHOULDER_Y"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714] 
    # self.motor_names += ["RC-E-base-joint", "RC-E-linear-joint"]; self.motor_power += [0.0, 0.0]    
    self.motor_names += ["R_ELBOW_P"]; self.motor_power += [247.16]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.813]
    # self.motor_names += ["R_ELBOW_Y"]; self.motor_power += [123.62]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.727]     
    # self.motor_names += ["R_WRIST_P"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]
    # self.motor_names += ["R_WRIST_Y"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]
    # self.motor_names += ["R_HAND"];    self.motor_power += [94.4];   self.pGainList+= [1];     self.vGainList+= [0.1]; self.maxVelList+= [675]
   
    # self.motor_names += ["L_CROTCH_Y"]; self.motor_power += [112.38];   self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [283.5]
    # self.motor_names += ["L_CROTCH_R"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]
    self.motor_names += ["L_CROTCH_P"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]   
    # self.motor_names += ["LC-CI-base-joint", "LC-CI-linear-joint", "LC-CI-hinge-joint", "LC-CO-base-joint" "LC-CO-linear-joint", "LC-CO-hinge-joint", "LC-K-base-joint", "LC-K-linear-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0, 0, 0]    
    self.motor_names += ["L_KNEE_P"];   self.motor_power += [477.9]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [200]             
    # self.motor_names += ["LC-AI-base-joint", "LC-AI-linear-joint", "LC-AI-hinge-joint", "LC-AO-base-joint" "LC-AO-linear-joint", "LC-AO-hinge-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0]     
    # self.motor_names += ["L_ANKLE_R"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667]     
    self.motor_names += ["L_ANKLE_P"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667] 
    
    # self.motor_names += ["R_CROTCH_Y"]; self.motor_power += [112.38];   self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [283.5]
    # self.motor_names += ["R_CROTCH_R"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]
    self.motor_names += ["R_CROTCH_P"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]   
    # self.motor_names += ["RC-CI-base-joint", "RC-CI-linear-joint", "RC-CI-hinge-joint", "RC-CO-base-joint" "RC-CO-linear-joint", "RC-CO-hinge-joint", "RC-K-base-joint", "RC-K-linear-joint"] ; self.motor_power += [0, 0, 0, 0, 0, 0, 0, 0]    
    self.motor_names += ["R_KNEE_P"];   self.motor_power += [477.9]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [200]             
    # self.motor_names += ["RC-AI-base-joint", "RC-AI-linear-joint", "RC-AI-hinge-joint", "RC-AO-base-joint" "RC-AO-linear-joint", "RC-AO-hinge-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0]              
    # self.motor_names += ["R_ANKLE_R"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667]    
    self.motor_names += ["R_ANKLE_P"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667] 

    self.motor_list = [self.jdict[n] for n in self.motor_names]
    if self.random_yaw:
      position = [0, 0, 0]
      orientation = [0, 0, 0]
      yaw = self.np_random.uniform(low=-3.14, high=3.14)
      if self.random_lean and self.np_random.randint(2) == 0:
        cpose.set_xyz(0, 0, 1.4)
        if self.np_random.randint(2) == 0:
          pitch = np.pi / 2
          position = [0, 0, 0.45]
        else:
          pitch = np.pi * 3 / 2
          position = [0, 0, 0.25]
        roll = 0
        orientation = [roll, pitch, yaw]
      else:
        position = [0, 0, 1.4]
        orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
      self.robot_body.reset_position(position)
      self.robot_body.reset_orientation(orientation)

  random_yaw = False; random_lean = False

  def apply_action(self, action):
    self.choreonoidPose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    # assert (np.isfinite(action).all()); assert (len(self.motor_list) == len(action))
    # print ("acion function", action)
    # f = open('/home/admin/dribble_repo/action_log.txt','a') # Prints Action Values
    for jointIndex, motorListIndex, powerIndex, pgain, vgain, velIndex, nameIndex in zip(range(len(self.motor_list)), self.motor_list, self.motor_power, self.pGainList, self.vGainList, self.maxVelList, self.motor_names):
      lo3, hi3= pybullet.getJointInfo(1,jointIndex)[8], pybullet.getJointInfo(1, jointIndex)[9]
      self.desired_position= np.clip(action[jointIndex], lo3, hi3)
      desired_torque= self.power*powerIndex
      maxVel= np.deg2rad(velIndex)
      self.desired_Vel= np.clip(action[jointIndex],-1*maxVel, maxVel)

      # if jointIndex>19: motorListIndex.set_position(initialPoseList2[jointIndex], desired_torque, maxVel)

      # if i==self.leftbutRightSR: motorListIndex.set_position(np.clip(action[i], lo3, 0), desired_torque, maxVel) #ACTUALLy RIGHT 
      # if i==self.rightButLeftSR: motorListIndex.set_position(np.clip(action[i], 0, hi3), desired_torque, maxVel) #ACTUALLY LEFT ARM
      
      # if jointIndex>19:#==self.leftAP or jointIndex==self.rightAP or jointIndex==self.rightAR or jointIndex==self.leftAR:
      #   #  motorListIndex.set_position(0, desired_torque, maxVel) #ACTUALLY LEFT ARM
      # # RSpACT=0;lcpACT=0;lkpACT=0; rep_ACT=0; lcyACT=0;lAP_act=0;lArACT=0
      # # if jointIndex==self.LButRSP: 
      
      #   motorListIndex.set_velocity(0.1* self.desired_Vel, desired_torque) 
      # else:
      # motorListIndex.set_velocity(1 * self.desired_Vel, desired_torque) 
      #   RSpACT = self.desired_position
      # if jointIndex==self.rightButLSP: motorListIndex.set_position(RSpACT, desired_torque, maxVel)
            
      # if jointIndex==self.rightEP: motorListIndex.set_position(self.desired_position, desired_torque, maxVel) 
      # rep_ACT = self.desired_position
      # if jointIndex==self.leftEP: motorListIndex.set_position(rep_ACT, desired_torque, maxVel)

      # if jointIndex==self.leftCP: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel) 
      #   lcpACT = self.desired_position
      # if jointIndex==self.rightCP: motorListIndex.set_position(lcpACT, desired_torque, maxVel)

      # if jointIndex==self.leftCY: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel) 
      #   lcyACT = self.desired_position
      # if jointIndex==self.rightCY: motorListIndex.set_position(-1*lcyACT, desired_torque, maxVel)

      # if jointIndex==self.leftCR: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel) 
      #   lcrACT = self.desired_position
      # if jointIndex==self.rightCR: motorListIndex.set_position(-1*lcrACT, desired_torque, maxVel)

      # # if jointIndex==self.rightCR: motorListIndex.set_position(-1 * action[self.leftCR], desired_torque, maxVel) #ACTUALLY LEFT ARM
      # if jointIndex==self.leftKP: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel)
      #   lkpACT = self.desired_position
      # if jointIndex==self.rightKP: motorListIndex.set_position(lkpACT, desired_torque, maxVel) #ACTUALLY LEFT ARM
 
      # if jointIndex==self.leftAR: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel) 
      #   lArACT = self.desired_position
      # if jointIndex==self.rightAR: motorListIndex.set_position(-1*lArACT, desired_torque, maxVel)

      # if jointIndex==self.leftAP: 
      #   motorListIndex.set_position(self.desired_position, desired_torque, maxVel)
      #   lAP_act = self.desired_position
      # if jointIndex==self.rightAP: motorListIndex.set_position(lAP_act, desired_torque, maxVel)     
      # # if jointIndex==self.rightAP: motorListIndex.set_position(action[self.leftAP], desired_torque, maxVel) #ACTUALLY LEFT ARM
      # # if jointIndex==self.rightAR: motorListIndex.set_position(-1 * action[self.leftAR], desired_torque, maxVel) #ACTUALLY LEFT ARM
      # # if jointIndex>19: motorListIndex.set_position(np.clip(action[jointIndex], 0, hi3), desired_torque, maxVel) #ACTUALLY LEFT ARM
      
      # if jointIndex==self.rightAP or jointIndex==self.leftAP: motorListIndex.set_position(0, desired_torque, maxVel)     
      # else:
      
      # motorListIndex.set_velocity(1 * self.desired_Vel, desired_torque) 
      motorListIndex.set_position(self.desired_position, desired_torque, maxVel)
      self.choreonoidPose[jointIndex] = self.desired_position
      # return action
      # motorListIndex.set_positionNoVel(desired_position, desired_torque)
      # motorListIndex.positionPD(self.desired_position, desired_torque, maxVel, pgain, vgain)
      # motorListIndex.pdControl(desired_position, desired_torque, pgain, vgain, maxVel)
      # f.write(str(desired_position)); f.write(" ")
      # self._p.setJointMotorControl2(1, jointIndex, pybullet.POSITION_CONTROL, targetPosition= desired_position, force=desired_torque, maxVelocity=maxVel)#, positionGain=pgain, velocityGain=vgain)
      # whatjoint = self.leftCP
      # print("joint: "+ self.motor_names[whatjoint])
    # print("LKP: ", action[self.leftKP], "RKP: ", lkpACT)
      #   print ("desposition for", desired_position)    
    # f.write("\n"); f.close()    
    # print ("acion after for", action)
  
  def feedbackCallback(self, msg):
      self.feedback = msg
  def jointCallback(self, joint_states):
    array46pos = np.array(joint_states.position)
    array46vel = np.array(joint_states.velocity)
    # the21 = [7, 14, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30]
    # self.statepos = array46pos[the21]; self.statevel = array46vel[the21]
  def manual_input(self):
      maiinput = input("put in radians: ")
      print (f"maiinput self.b: {maiinput}")
      if not maiinput: print ("no iMPut")
      return maiinput
  def choreonoidJoints(self, fullbody, goal):
      # goal.trajectory.joint_names.extend(["CHEST_Y", "CHEST_P"]) #0 #1
      # goal.trajectory.joint_names.extend(["HEAD_Y", "HEAD_P"]) #2 #3
      goal.trajectory.joint_names.extend(["L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y"]) #4 #5 #6
      goal.trajectory.joint_names.extend(["L_ELBOW_P"]) #, "L_ELBOW_Y"])#7 #8
      # goal.trajectory.joint_names.extend(["L_WRIST_P", "L_WRIST_Y", "LH_state"])#9 #10 #11

      goal.trajectory.joint_names.extend(["R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y"]) #12 #13 #14
      goal.trajectory.joint_names.extend(["R_ELBOW_P"]) #, "R_ELBOW_Y"])#15 #16
      # goal.trajectory.joint_names.extend(["R_WRIST_P", "R_WRIST_Y", "RH_state"]) #17 #18 #19

      goal.trajectory.joint_names.extend(["L_CROTCH_P", "L_KNEE_P", "L_ANKLE_P"]) #, "L_CROTCH_P"]) #20 #21 #22
      goal.trajectory.joint_names.extend(["R_CROTCH_P", "R_KNEE_P", "R_ANKLE_P"])

      # goal.trajectory.joint_names.extend(["L_CROTCH_Y", "L_CROTCH_R", "L_CROTCH_P"]) #20 #21 #22
      # goal.trajectory.joint_names.extend(["L_KNEE_P", "L_ANKLE_R", "L_ANKLE_P"]) #23 #24 #25
      # goal.trajectory.joint_names.extend(["R_CROTCH_Y", "R_CROTCH_R", "R_CROTCH_P"]) #26 #27 #28
      # goal.trajectory.joint_names.extend(["R_KNEE_P", "R_ANKLE_R", "R_ANKLE_P"]) #29 #30 #31

      point = JointTrajectoryPoint()
      # g = self.initialPoseList2
      # if not g[0]: # len(g[0]) == 0:
          # break
      # print (f"JOINTS ACTIVE: {goal.trajectory.joint_names}")
      # print (f"g = self.functionforinput() = {g}")
      # print (f"length of list [g]: {len(g)}") #print (f"g: {g}")
      point.positions = [float(x) for x in self.choreonoidPose]
      # print(f"point.positions input = {point.positions}")
      point.time_from_start = rospy.Duration(0.99)
      # print("rospy Duration line worked")
      # point.velocities = [0.0] * len(point.positions)
      # point.accelerations = [0.0] * len(point.positions)
      # point.effort = [0.0] * len(point.positions)
      goal.trajectory.points.append(point)
      # print ("goal trajectory append points worked")
      fullbody.send_goal(goal, feedback_cb = self.feedbackCallback)
      # self.rate.sleep()
      # print ("send_goal line worked")
      # fullbody.wait_for_result() #Blocks until this goal transitions to done.
      # print("wait for result line SKIPPED")
      # print (f"state from other class: {self.calc_state()} ")
      # self.rate.sleep()

  def rosBridgeInitNode(self):
      rospy.init_node('stand_Node')    
      rospy.Subscriber('/joint_states', JointState, self.jointCallback)

  def choreonoidOut(self):
      # print (self.desired_position)
      # self.rate = rospy.Rate(300)    
      # while not rospy.is_shutdown():
      fullbody = actionlib.SimpleActionClient("/fullbody_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
      self.choreonoidJoints(fullbody, FollowJointTrajectoryGoal())      

