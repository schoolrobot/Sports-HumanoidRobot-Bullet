#31 all 32 link states position quaternions

from drb_robot_bases import URDFBasedRobot#, BodyPart
import numpy as np
import pybullet, pybullet_data, os, rospy, actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class WalkerBaseURDF(URDFBasedRobot):

  def __init__(self, fn, robot_name, action_dim, obs_dim, power):
    URDFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim)
    self.power = power
    # self.camera_x = 0
    # self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
    self.walk_target_x = 0.01  # kilometer away
    self.walk_target_y = 0
    # self.waistXYZ = [0, 0, 0]
    self.base_id, self.chestY_id, self.chestP_id= -1, 0, 1
    self.headY_id, self.headP_id= 2, 3

    self.LButRSP, self.leftbutRightSR, self.leftButRSY= 4, 5, 6 #SHOULDER PAD // BONE // BIandTRICEP
    self.rightButLSP, self.rightButLeftSR, self.rightSY= 12, 13, 14
    self.leftEP, self.leftEY= 7, 8 #ELBOW PAD // FOREARM
    self.rightEP, self.rightEY= 15, 16
    self.leftWP, self.leftWY, self.leftH= 9, 10, 11 #HAND SENSOR // KNUCKLE// FINGERS
    self.rightWP, self.rightWY, self.rightH= 17, 18, 19

    self.leftCY, self.leftCR, self.leftCP= 20, 21, 22 # ABOVE QAUD // TINY BALL // THIGH 
    self.rightCY, self.rightCR, self.rightCP= 26, 27, 28
    self.leftKP, self.rightKP= 23, 29
    self.leftAR, self.leftAP= 24, 25
    self.rightAR, self.rightAP= 29, 31

  def setup_collision_group(self, link_a_ids, link_b_ids, flag):
      [self.setup_collision(i, j, flag) for i in link_a_ids for j in link_b_ids]

  def setup_collision(self, link_a, link_b, flag):
      if link_a != link_b: pybullet.setCollisionFilterPair(1, 1, link_a, link_b, flag)

  def noCollision(self):
    BODY_LINK_IDS = tuple(range(-1, 4))# + (16, 17))
    # ARM4BODY_LINK_IDS = tuple((4, 17) + tuple(range(18, 27)))
    L_ARM_LINK_IDS = tuple(range(4, 12))
    R_ARM_LINK_IDS = tuple(range(12, 20))
    L_LEG_LINK_IDS = tuple(range(20, 26))
    R_LEG_LINK_IDS = tuple(range(26, 31))
    all_link_ids = tuple(range(-1,33))    
    self.setup_collision_group(all_link_ids, all_link_ids, False)
    self.setup_collision_group(L_ARM_LINK_IDS, BODY_LINK_IDS, True)
    self.setup_collision_group(R_ARM_LINK_IDS, BODY_LINK_IDS, True)
    self.setup_collision_group(R_ARM_LINK_IDS, L_ARM_LINK_IDS, True)
    self.setup_collision_group(R_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
    self.setup_collision_group(R_ARM_LINK_IDS, L_LEG_LINK_IDS, True)
    self.setup_collision_group(L_ARM_LINK_IDS, L_LEG_LINK_IDS, True)    
    self.setup_collision_group(L_ARM_LINK_IDS, R_LEG_LINK_IDS, True)
    self.setup_collision_group(L_LEG_LINK_IDS, R_LEG_LINK_IDS, True)
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

  def printTime(self):
    self.t+=self.dt
    # txtid = pybullet.addUserDebugText("time="+str(self.t), [0,0,2],replaceItemUniqueId=self.timeid)
    print ("time = ", self.t)
  def myResetFunc(self):
    self.colorTF()
    floorFriction= 1
    pybullet.changeDynamics(0, -1, mass=0 
                            ,lateralFriction= floorFriction, rollingFriction=floorFriction, spinningFriction=floorFriction 
                            ,linearDamping=1, angularDamping=1 
                            ,contactDamping=-1 ,contactStiffness=-1
                            ,restitution= 0
                            ,frictionAnchor=1)
    # pybullet.enableJointForceTorqueSensor(1, self.leftAP, enableSensor= 1)
    # pybullet.enableJointForceTorqueSensor(1, self.rightAP, enableSensor= 1)
    # pybullet.enableJointForceTorqueSensor(1, self.rightWP, enableSensor= 1)
    # pybullet.enableJointForceTorqueSensor(1, self.leftWP, enableSensor= 1)

    pybullet.changeVisualShape(1, self.rightCP , rgbaColor= [0.2,0.6, 1,1])
    pybullet.changeVisualShape(1, self.leftCY, rgbaColor= [0,1, 0,1])
    pybullet.changeVisualShape(1, self.leftCP , rgbaColor= [1,0,0,0.7])
    pybullet.changeVisualShape(1, self.leftEY, rgbaColor= [1, 0.2, 0.7, 1])
    pybullet.changeVisualShape(1, self.rightEY, rgbaColor= [0.29, 0.29, 0.28, 1])
    pybullet.changeVisualShape(1, self.rightEP, rgbaColor= [0.28, 0.28, 0.28, 1])
    pybullet.changeVisualShape(1, self.leftButRSY, rgbaColor= [0.2, 0.2, 0.2, 1])

  def robot_specific_reset(self, bullet_client):
    pybullet = bullet_client
    self.t = 0

    self.no_of_steps= 0
    initialPoseList= [
      0, #0 CHEST_Y
      0, #1 CHEST_P
      0, #2 HEAD_Y
      0, #3 HEAD_P

      0.2618,  #4 L_SHOULDER_P
      -0.2618, #5 L_SHOULDER_R #flips -/+ by itself (originally) +
      -0.0873, #6 L_SHOULDER_Y
      -0.5236, #7 L_ELBOW_P
      0, #8 L_ELBOW_Y
      0, #9 L_WRIST_P
      0, #10 L_WRIST_Y
      0, #-0.5061, #11 LH_state

      0.2618, #12 R_SHOULDER_P
      0.2618, #13 R_SHOULDER_R #flips -/+ by itself (originally) -
      0.0873, #14 R_SHOULDER_Y
      -0.5236, #15 R_ELBOW_P
      0, #16 R_ELBOW_Y
      0, #17 R_WRIST_P
      0, #18 R_WRIST_Y
      0, #-0.5061, #19 RH_state

      0, #20 L_CROTCH_Y
      0.1745, #21 L_CROTCH_R
      -0.2967, #22 L_CROTCH_P
      0.6283, #23 L_KNEE_P
      -0.1745, #24 L_ANKLE_R
      -0.2967, #25 L_ANKLE_P

      0, #26 R_CROTCH_Y
      -0.1745, #27 R_CROTCH_R
      -0.2967, #28 R_CROTCH_P
      0.6283, #29 R_KNEE_P
      0.1745, #30 R_ANKLE_R
      -0.2967 #31 R_ANKLE_P
    ]
    # print (self.ordered_joints)
    for i, j in zip(range(30),self.ordered_joints): #zip range because joint type not iterable
    # for i in range(pybullet.getNumJoints(1)):
      lo, hi= pybullet.getJointInfo(1,i)[8], pybullet.getJointInfo(1, i)[9]
      footFriction=1; footDamping=1; damping= 3
      if i==self.leftAR or i==self.rightAR: pybullet.changeDynamics(1, i
                                            ,lateralFriction=footFriction, rollingFriction=footFriction, spinningFriction=footFriction
                                            ,linearDamping=footDamping, angularDamping=footDamping
                                            ,contactDamping=-1 ,contactStiffness=-1 
                                            ,restitution=0
                                            ,frictionAnchor=1 ,contactProcessingThreshold=1
                                            )
      else: pybullet.changeDynamics(1, i, linearDamping=1, angularDamping=1, jointDamping=1, restitution=0)
      j.disable_motor()
      j.reset_current_position(0, 0)
      # j.reset_current_position(initialPoseList[i], 0)
      # j.change_d(1, 10)

      # if i >19: pybullet.changeDynamics(1, i, lateralFriction=1, rollingFriction=1, spinningFriction=1, linearDamping=damping, angularDamping=damping)
      # pybullet.changeDynamics(1, i, maxJointVelocity=self.max)
      # if i==self.leftKP or i==self.rightKP: pybullet.changeDynamics(1, i, linearDamping=3, angularDamping=3)
      # if i==self.leftCP or i==self.rightCP: pybullet.changeDynamics(1, i, linearDamping=3, angularDamping=3)
      # pybullet.resetJointState(1, i, initialPoseList[i])
      # pybullet.resetJointState(1, i, 0)#np.random.uniform(low=-0.1, high=0.1))      
      # j.reset_current_position(initialPoseList[indexForReset], 0) #TypeError must be real number not list
      # if i!=self.leftbutRightSR and i!=self.rightButLeftSR:

      # if i==self.leftbutRightSR: j.reset_current_position(np.random.uniform(low=lo,high=0), 0) #ACTUALLy RIGHT 
      # if i==self.rightButLeftSR: j.reset_current_position(np.random.uniform(low=0,high=hi), 0) #ACTUALLY LEFT ARM      
      # if i==self.LButRSP or i==self.rightButLSP: j.reset_current_position(np.random.uniform(low=lo,high=hi),0) 
      # if i==self.leftButRSY or i==self.rightSY: j.reset_current_position(np.random.uniform(low=lo,high=hi),0) 
      # if i==self.leftEP or i==self.rightEP: j.reset_current_position(np.random.uniform(low=lo,high=hi),0) 
      # if i==self.leftEY or i==self.rightEY: j.reset_current_position(np.random.uniform(low=lo,high=hi),0)
      # if i==self.leftWP or i==self.rightWP: j.reset_current_position(np.random.uniform(low=lo, high=hi),0)
      # if i==self.leftWY or i==self.rightWY: j.reset_current_position(np.random.uniform(low=lo,high=hi),0)
      # if i==self.leftH or i==self.rightH: j.reset_current_position(np.random.uniform(low=lo,high=hi),0) 
      
      # if i==self.chestP_id or i==self.headP_id: j.reset_current_position(np.random.uniform(low=lo,high=hi),0) # 
      # if i==self.headY_id: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=hi))
      # if i==self.chestY_id: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=hi))
      
      # if i==self.leftKP or i==self.rightKP: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=hi))
      # if i==self.leftCP or i==self.rightCP: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=hi))
      # if i==self.rightCR: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=0))
      # if i==self.leftCR: pybullet.resetJointState(1, i, np.random.uniform(low=0,high=hi))
      # if i==self.leftAR: pybullet.resetJointState(1, i, np.random.uniform(low=0,high=hi), 0)
      # if i==self.rightAR: pybullet.resetJointState(1, i, np.random.uniform(low=lo,high=0), 0)
      # if i==self.leftAP or i==self.rightAP: pybullet.resetJointState(1, i, np.random.uniform(low=-1.66,high=hi))
      # if i==self.leftCY: pybullet.resetJointState(1, i, np.random.uniform(low=-0.35,high=0.51))
      # if i==self.rightCY: pybullet.resetJointState(1, i, np.random.uniform(low=-0.51,high=0.35))

      # if i>19: j.reset_current_position(initialPoseList[i], 0)#np.random.uniform(low=lo,high=hi))
      # elif i<4: j.reset_current_position(initialPoseList[i],0)# initialPoseList[i])
      # elif i==self.leftbutRightSR: j.reset_current_position(-0.08726,0)#np.random.uniform(low=0.05*lo,high=0), 0) #pybullet_RIGHT lo is ext
      # elif i==self.rightButLeftSR: j.reset_current_position(0.08726,0)#np.random.uniform(low=0,high=hi), 0) #pyb LEFT ARM       
      # else: j.reset_current_position(np.random.uniform(size=1,low=lo, high=hi),0)# initialPoseList[i])

      # pybullet.resetJointState(1, i, initialPoseList[i])
    self.initHeadH= pybullet.getLinkState(1,self.headP_id)[0][2]
    self.initChestPH= pybullet.getLinkState(1,self.chestP_id)[0][2]
    self.initChestY= pybullet.getLinkState(1,self.chestY_id)
    self.initWaistH= pybullet.getBasePositionAndOrientation(1)[0][2]  
    self.initLKnee= pybullet.getLinkState(1, self.leftKP)[0][2]
    self.initRKnee= pybullet.getLinkState(1, self.rightKP)[0][2]

    # self.feet = [self.link_dict[f] for f in self.foot_list]
    # self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
    self.feet_on_ground=np.array([0 for i in self.foot_list], dtype=np.float32) 
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

  def calc_state(self):
    self.no_of_steps += 1
    # self.t+=1/400
    # print ("time in calc_state = ", self.t)    

    # posList, velList= [], []
    # for i in range(pybullet.getNumJoints(1)):
    #   lo2, hi2= pybullet.getJointInfo(1,i)[8], pybullet.getJointInfo(1, i)[9]
    #   pos, vel = pybullet.getJointState(1, i)[0], pybullet.getJointState(1, i)[1]
    #   pos_mid = 0.5 * (lo2 + hi2)
    #   relativePos= (2 * (pos - pos_mid) / (hi2 - lo2))    
    #   posList.append(relativePos), velList.append(vel)
    jointPV = np.array([j.current_relative_position() for j in self.ordered_joints]).flatten()#, dtype=np.float32).flatten()
    # jointPV = np.array([j.ROS_relative_position() for j in self.ordered_joints]).flatten()#, dtype=np.float32).flatten()
    
    # even elements [0::2] position, scaled to -1..+1 between limits
    # odd elements  [1::2] angular speed, scaled to show -1..+1
    self.joint_speeds =jointPV[1::2] #velList
    self.joints_at_limit = np.count_nonzero(np.abs(jointPV[0::2]) > 0.99)
    # print(j[0::2])
    
    self.waistState= pybullet.getBasePositionAndOrientation(1); self.waistXYZ = self.waistState[0]; waist_quat = self.waistState[1]; self.waistAngleEuler = pybullet.getEulerFromQuaternion(waist_quat)    
    self.chestY_state= pybullet.getLinkState(1,self.chestY_id); self.chestY_XYZ= self.chestY_state[0]; chestY_quat= self.chestY_state[1]
    self.chestP_state= pybullet.getLinkState(1,self.chestP_id); self.chestP_XYZ= self.chestP_state[0]; chestP_quat= self.chestP_state[1]
    self.headY_state= pybullet.getLinkState(1,self.headY_id);   self.headY_XYZ= self.headY_state[0]; headY_quat= self.headY_state[1]
    self.headP_state= pybullet.getLinkState(1,self.headP_id);   self.headP_XYZ= self.headP_state[0]; headP_quat= self.headP_state[1]

    self.LSP_state= pybullet.getLinkState(1,self.LButRSP); self.LSP_XYZ= self.LSP_state[0]; LSP_quat= self.LSP_state[1]
    self.LSR_state= pybullet.getLinkState(1,self.rightButLeftSR); self.LSR_XYZ= self.LSR_state[0]; LSR_quat= self.LSR_state[1]
    self.LSY_state= pybullet.getLinkState(1,self.leftButRSY); self.LSY_XYZ= self.LSY_state[0]; LSY_quat= self.LSY_state[1]
    self.LEP_state= pybullet.getLinkState(1,self.leftEP); self.LEP_XYZ= self.LEP_state[0]; LEP_quat= self.LEP_state[1]
    self.LEY_state= pybullet.getLinkState(1,self.leftEY); self.LEY_XYZ= self.LEY_state[0]; LEY_quat= self.LEY_state[1]
    self.LWP_state= pybullet.getLinkState(1,self.leftWP); self.LWP_XYZ= self.LWP_state[0]; LWP_quat= self.LWP_state[1]
    self.LWY_state= pybullet.getLinkState(1,self.leftWY); self.LWY_XYZ= self.LWY_state[0]; LWY_quat= self.LWY_state[1]
    self.LH_state= pybullet.getLinkState(1,self.leftH); self.LH_XYZ= self.LH_state[0]; LH_quat= self.LH_state[1]

    self.RSP_state= pybullet.getLinkState(1,self.rightButLSP); self.RSP_XYZ= self.RSP_state[0]; RSP_quat= self.RSP_state[1]
    self.RSR_state= pybullet.getLinkState(1,self.leftbutRightSR); self.RSR_XYZ= self.RSR_state[0]; RSR_quat= self.RSR_state[1]
    self.RSY_state= pybullet.getLinkState(1,self.rightSY); self.RSY_XYZ= self.RSY_state[0]; RSY_quat= self.RSY_state[1]
    self.REP_state= pybullet.getLinkState(1,self.rightEP); self.REP_XYZ= self.REP_state[0]; REP_quat= self.REP_state[1]
    self.REY_state= pybullet.getLinkState(1,self.rightEY); self.REY_XYZ= self.REY_state[0]; REY_quat= self.REY_state[1]
    self.RWP_state= pybullet.getLinkState(1,self.rightWP); self.RWP_XYZ= self.RWP_state[0]; RWP_quat= self.RWP_state[1]
    self.RWY_state= pybullet.getLinkState(1,self.rightWY); self.RWY_XYZ= self.RWY_state[0]; RWY_quat= self.RWY_state[1]
    self.RH_state= pybullet.getLinkState(1,self.rightH); self.RH_XYZ= self.RH_state[0]; RH_quat= self.RH_state[1]

    self.LCY_state= pybullet.getLinkState(1,self.leftCY); self.LCY_XYZ= self.LCY_state[0]; LCY_quat= self.LCY_state[1]
    self.LCR_state= pybullet.getLinkState(1,self.leftCR); self.LCR_XYZ= self.LCR_state[0]; LCR_quat= self.LCR_state[1]
    self.LCP_state= pybullet.getLinkState(1,self.leftCP); self.LCP_XYZ= self.LCP_state[0]; LCP_quat= self.LCP_state[1]
    self.LKP_state= pybullet.getLinkState(1,self.leftKP); self.LKP_XYZ= self.LKP_state[0]; LKP_quat= self.LKP_state[1]
    self.LAR_state= pybullet.getLinkState(1,self.leftAR); self.LAR_XYZ= self.LAR_state[0]; LAR_quat= self.LAR_state[1]
    # self.LAP_state= pybullet.getLinkState(1,self.leftAP,1); self.LAP_XYZ= self.LAP_state[0]; LAP_quat = self.LAP_state[1]; self.LAP_rad= pybullet.getEulerFromQuaternion(LAP_quat)

    self.RCY_state= pybullet.getLinkState(1,self.rightCY); self.RCY_XYZ= self.RCY_state[0]; RCY_quat= self.RCY_state[1]
    self.RCR_state= pybullet.getLinkState(1,self.rightCR); self.RCR_XYZ= self.RCR_state[0]; RCR_quat= self.RCR_state[1]
    self.RCP_state= pybullet.getLinkState(1,self.rightCP); self.RCP_XYZ= self.RCP_state[0]; RCP_quat= self.RCP_state[1]
    self.RKP_state= pybullet.getLinkState(1,self.rightKP); self.RKP_XYZ= self.RKP_state[0]; RKP_quat= self.RKP_state[1]
    self.RAR_state= pybullet.getLinkState(1,self.rightAR); self.RAR_XYZ= self.RAR_state[0]; RAR_quat= self.RAR_state[1]
    # self.RAP_state= pybullet.getLinkState(1,self.rightAP,1); self.RAP_XYZ= self.RAP_state[0]; RAP_quat = self.RAP_state[1]; self.RAP_rad= pybullet.getEulerFromQuaternion(RAP_quat)

    # self.LAP_linearV = self.LAP_state[6]; self.RAP_linearV = self.RAP_state[6]
    # self.leftKnee_X= pybullet.getLinkState(1,self.leftKP)[0][0]
    # self.leftAnkleP_X= pybullet.getLinkState(1,self.leftAP)[0][0]
    # print ("chest P", self.chestP_XYZ[2]); # print ("chest Y", self.chestY_state[0][2]); # print ("waist Z", self.waistXYZ[2])

    roll, pitch, yaw = self.waistAngleEuler
    self.walk_target_theta = np.arctan2(self.walk_target_y - self.waistXYZ[1], self.walk_target_x - self.waistXYZ[0])
    self.walk_target_dist = np.linalg.norm([self.walk_target_y - self.waistXYZ[1], self.walk_target_x - self.waistXYZ[0]])
    angle_to_target = self.walk_target_theta - yaw
    rot_speed = np.array([[np.cos(-yaw), -np.sin(-yaw), 0], [np.sin(-yaw), np.cos(-yaw), 0], [0, 0, 1]])
    vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

    # lFSensor= pybullet.getJointState(1, self.leftAP)[2]; rFSensor= pybullet.getJointState(1, self.rightAP)[2]
    # lHandSensor= pybullet.getJointState(1, self.leftWY)[2]; rHandSensor= pybullet.getJointState(1, self.rightWY)[2]
    # print("left foot Fz", int(rFSensor[0]))
    waistVel= pybullet.getBaseVelocity(1)
    # print("BASE VELOCITY |", "X:", format(waistVel[0][0],".2f"), "  Y:", format(waistVel[0][1],".2f"), "  Z:", format(waistVel[0][2],".2f"))
    # print("LEFT FOOT VELOCITY |", "X:", format(self.LAP_state[6][0],".2f"), "  Y:", format(self.LAP_state[6][1],".2f"), "  Z:", format(self.LAP_state[6][2],".2f"))
    # print ("LEFT_AP_STATE:", self.LAP_state[6])

    # more = np.array(
    #     [ z - self.initial_z,
    #       # np.sin(angle_to_target), np.cos(angle_to_target),
    #       # 0.3 * vx, 0.3 * vy, 0.3 * vz,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
    #       # roll, pitch
    #     ], dtype=np.float32)
    
    torso_XYZ_quat = np.array([
      self.waistXYZ[0] ,self.waistXYZ[1], self.waistXYZ[2] ,waist_quat[0] ,waist_quat[1], waist_quat[2]      
      ,self.chestY_XYZ[0], self.chestY_XYZ[1] ,self.chestY_XYZ[2] ,chestY_quat[0], chestY_quat[1] ,chestY_quat[2]
      ,self.chestP_XYZ[0], self.chestP_XYZ[1] ,self.chestP_XYZ[2] ,chestP_quat[0], chestP_quat[1] ,chestP_quat[2]
      ,self.headY_XYZ[0], self.headY_XYZ[1] ,self.headY_XYZ[2] ,headY_quat[0], headY_quat[1] ,headY_quat[2]      
      ,self.headP_XYZ[0], self.headP_XYZ[1] ,self.headP_XYZ[2] ,headP_quat[0], headP_quat[1] ,headP_quat[2]      
      ], dtype=np.float32 )

    L_arm_XYZ_quat = np.array([
      self.LSP_XYZ[0] ,self.LSP_XYZ[1], self.LSP_XYZ[2] ,LSP_quat[0], LSP_quat[1], LSP_quat[2]
      ,self.LSR_XYZ[0] ,self.LSR_XYZ[1], self.LSR_XYZ[2] ,LSR_quat[0], LSR_quat[1], LSR_quat[2]
      ,self.LSY_XYZ[0] ,self.LSY_XYZ[1], self.LSY_XYZ[2] ,LSY_quat[0], LSY_quat[1], LSY_quat[2]      
      ,self.LEP_XYZ[0] ,self.LEP_XYZ[1], self.LEP_XYZ[2] ,LEP_quat[0], LEP_quat[1], LEP_quat[2]
      ,self.LEY_XYZ[0], self.LEY_XYZ[1] ,self.LEY_XYZ[2] ,LEY_quat[0], LEY_quat[1], LEY_quat[2]
      ,self.LWP_XYZ[0], self.LWP_XYZ[1] ,self.LWP_XYZ[2], LWP_quat[0], LWP_quat[1], LWP_quat[2]
      ,self.LWY_XYZ[0], self.LWY_XYZ[1] ,self.LWY_XYZ[2], LWY_quat[0], LWY_quat[1], LWY_quat[2]
      ,self.LH_XYZ[0], self.LH_XYZ[1] ,self.LH_XYZ[2], LH_quat[0], LH_quat[1], LH_quat[2]
      ], dtype=np.float32 )

    R_arm_XYZ_quat = np.array([
      self.RSP_XYZ[0] ,self.RSP_XYZ[1], self.RSP_XYZ[2] ,RSP_quat[0], RSP_quat[1], RSP_quat[2]
      ,self.RSR_XYZ[0] ,self.RSR_XYZ[1], self.RSR_XYZ[2] ,RSR_quat[0], RSR_quat[1], RSR_quat[2]
      ,self.RSY_XYZ[0] ,self.RSY_XYZ[1], self.RSY_XYZ[2] ,RSY_quat[0], RSY_quat[1], RSY_quat[2]      
      ,self.REP_XYZ[0] ,self.REP_XYZ[1], self.REP_XYZ[2] ,REP_quat[0], REP_quat[1], REP_quat[2]
      ,self.REY_XYZ[0], self.REY_XYZ[1] ,self.REY_XYZ[2] ,REY_quat[0], REY_quat[1], REY_quat[2]
      ,self.RWP_XYZ[0], self.RWP_XYZ[1] ,self.RWP_XYZ[2] ,RWP_quat[0], RWP_quat[1], RWP_quat[2]
      ,self.RWY_XYZ[0], self.RWY_XYZ[1] ,self.RWY_XYZ[2] ,RWY_quat[0], RWY_quat[1], RWY_quat[2]
      ,self.RH_XYZ[0], self.RH_XYZ[1] ,self.RH_XYZ[2] ,RH_quat[0], RH_quat[1], RH_quat[2]
      ], dtype=np.float32 )

    L_leg_XYZ_quat = np.array([
      self.LCY_XYZ[0], self.LCY_XYZ[1] ,self.LCY_XYZ[2] ,LCY_quat[0], LCY_quat[1], LCY_quat[2]
      ,self.LCR_XYZ[0], self.LCR_XYZ[1] ,self.LCR_XYZ[2] ,LCR_quat[0], LCR_quat[1], LCR_quat[2]
      ,self.LCP_XYZ[0], self.LCP_XYZ[1] ,self.LCP_XYZ[2] ,LCP_quat[0], LCP_quat[1], LCP_quat[2]
      ,self.LKP_XYZ[0], self.LKP_XYZ[1] ,self.LKP_XYZ[2] ,LKP_quat[0], LKP_quat[1], LKP_quat[2]      
      ,self.LAR_XYZ[0], self.LAR_XYZ[1] ,self.LAR_XYZ[2] ,LAR_quat[0], LAR_quat[1], LAR_quat[2]  
      # ,self.LAP_XYZ[0], self.LAP_XYZ[1] ,self.LAP_XYZ[2] ,LAP_quat[0], LAP_quat[1], LAP_quat[2]  
      ], dtype=np.float32 )

    R_leg_XYZ_quat = np.array([
      self.RCY_XYZ[0], self.RCY_XYZ[1] ,self.RCY_XYZ[2] ,RCY_quat[0], RCY_quat[1], RCY_quat[2]
      ,self.RCR_XYZ[0], self.RCR_XYZ[1] ,self.RCR_XYZ[2] ,RCR_quat[0], RCR_quat[1], RCR_quat[2]
      ,self.RCP_XYZ[0], self.RCP_XYZ[1] ,self.RCP_XYZ[2] ,RCP_quat[0], RCP_quat[1], RCP_quat[2]
      ,self.RKP_XYZ[0], self.RKP_XYZ[1] ,self.RKP_XYZ[2] ,RKP_quat[0], RKP_quat[1], RKP_quat[2]
      ,self.RAR_XYZ[0], self.RAR_XYZ[1] ,self.RAR_XYZ[2] ,RAR_quat[0], RAR_quat[1], RAR_quat[2]      
      # ,self.RAP_XYZ[0], self.RAP_XYZ[1] ,self.RAP_XYZ[2] ,RAP_quat[0], RAP_quat[1], RAP_quat[2]     
      ], dtype=np.float32 )

    # AP_reactionForce = np.array([ #SIX VALUES
    #   lFSensor[0], lFSensor[1] ,lFSensor[2] 
    #   ,rFSensor[0], rFSensor[1] ,rFSensor[2]# Z force
    #   #lHandSensor[0], lHandSensor[1],lHandSensor[2], # rHandSensor[0], rHandSensor[1], rHandSensor[2]
    #   ], dtype=np.int32 )

    chestY_Gsensor= np.array([
      chestY_quat[0], chestY_quat[1], chestY_quat[2] 
      ], dtype=np.float32 )

    # feetOrient= np.array([
    #   LAP_quat[0], LAP_quat[1] ,LAP_quat[2],
    #   RAP_quat[0], RAP_quat[1], RAP_quat[2]
    #    ], dtype=np.float32 )
    
    # feetLinerVel= np.array([
    #   self.LAP_linearV[0] ,self.RAP_linearV[0]
    #   ,self.LAP_linearV[1] ,self.RAP_linearV[1]
    #   ,self.LAP_linearV[2] ,self.RAP_linearV[2]
    # ])
    # print ("jay for ",1,1000*j[1])
    # print ("self.no_of_steps: ", self.no_of_steps)
    # print ("bodyRPY:",self.waistAngleEuler)
    # print ("J", [j])
    # return np.array(posList) #, velList)
    # print ("init left knee height:", self.LKP_state[0][2])
    # print ("init RIGHT knee height:", self.LCP_state[0][2])
    # print("chesty vwhy", chestY_Gsensor-pEE) 
    # print("chesty peee", pEE)

    return np.concatenate(
                        [jointPV] + #64
                        # + [chestY_Gsensor] #3+ [feetOrient] 
                        [torso_XYZ_quat] + [L_arm_XYZ_quat] + [R_arm_XYZ_quat] + [L_leg_XYZ_quat] + [R_leg_XYZ_quat] #198
                        # + [AP_reactionForce] #6
                        # + [self.feet_on_ground] 
                        )
    # return np.clip(np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

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

  def bodyPartHeights(self):
    headAboveChestP, headAbovewAIST, chestAboveWaist,crotchPAboveKnee, WaistAboveKnee, WaistAboveCrotch, waistKneesAligned, kneeAboveFoot= 0,0,0,0,0,0,0,0
    botchCrotchPPs= [self.LCP_state[0][2], self.RCP_state[0][2]]
    bothKneeHeights = [self.LKP_state[0][2], self.RKP_state[0][2]]
    bothFeetHeights= [self.LAP_state[0][2], self.RAP_state[0][2]]
    # if self.headP_XYZ[2] < self.chestP_XYZ[2]: headAboveChestP= -1  ;print("HEAD < below  CHEST P")
    if self.headP_XYZ[2] <= self.waistXYZ[2]: headAbovewAIST= -1  ;print("HEAD < below  waist")
    # if self.chestP_XYZ[2] < self.waistXYZ[2]: chestAboveWaist= -1   ;print("CHEST below WAIST")
    if self.waistXYZ[2] <= min(botchCrotchPPs): WaistAboveCrotch= -1   ;print("WAIST below CROTCH P")
    # if self.waistXYZ[2] < max(bothKneeHeights): WaistAboveKnee= -1   ;print("WAIST below KNEEs")
    if min(botchCrotchPPs) < min(bothKneeHeights): crotchPAboveKnee= -1   ;print("CROTCH PPPP below KNEEE")
    if min(bothKneeHeights) < min(bothFeetHeights): kneeAboveFoot= -1   ;print("knEEs below FEET")
   # if (self.waistXYZ[0]> max([self.LAP_state[0][0], self.RAP_state[0][0]])): WaistAboveKnee= -1   ;print("WAIST behind/front of Feet X")
   # if self.LKP_state[0][2] > self.initLKnee : X= -1 ; print("KNEES dropped")
    # if (self.waistXYZ[0] - self.headP_XYZ[0]) > 0.15: waistKneesAligned= -1 ; print("Head/Waist NOT ALIGNED")
    bodyPartHeightReward= [headAbovewAIST, WaistAboveCrotch, crotchPAboveKnee, kneeAboveFoot]
    # print("left knee now", float(self.LKP_state[0][2]), "left knee init", float(self.initLKnee))
    return +2 if sum(bodyPartHeightReward)== 0 else -1

  def linkInfo(self):
    return [self.headP_XYZ ,self.waistXYZ, self.LAP_state[0], self.RAP_state[0]]

  def imuZero(self):
    rollCost, pitchCost= 0, 0
    rollRad, pitchRad= pybullet.getEulerFromQuaternion(self.chestY_state[1])[0], pybullet.getEulerFromQuaternion(self.chestY_state[1])[1]
    headrollDeg, headpitchDeg= np.rad2deg(pybullet.getEulerFromQuaternion(self.headP_state[1])[0]), np.rad2deg(pybullet.getEulerFromQuaternion(self.headP_state[1])[1])
    rollDeg, pitchDeg = np.rad2deg(rollRad), np.rad2deg(pitchRad)
    rollCost= (2 * np.exp( -0.75 * (rollDeg/15)**2))-1
    if pitchDeg>=0:pitchCost= (2 * np.exp( -0.75 * (pitchDeg/30)**2))-1
    else: pitchCost= (2 * np.exp( -5 * (pitchDeg/15)**2))-1
    # pitchCost = (3 * np.exp( -0.7 * (pitchDeg/15)**2))-1.5
    # print ("PITCH",format(pitchDeg,".2f"),"ROLL", format(rollDeg,".2f"))
    return rollCost, pitchCost

  def imuAlive(self):
    rollRad, pitchRad= pybullet.getEulerFromQuaternion(self.chestY_state[1])[0], pybullet.getEulerFromQuaternion(self.chestY_state[1])[1]
    rollDeg, pitchDeg = np.rad2deg(rollRad), np.rad2deg(pitchRad)   
    # if abs(rollDeg) > 45: print("TOO MUCH ROOOLLLLLL")
    # if abs(pitchDeg) > 45: print("PIIIIITCH")
    leftFootWaistX = self.LAP_state[0][0] - self.waistXYZ[0]
    rightFootWaistX = self.RAP_state[0][0] - self.waistXYZ[0]
    feetXalive = (leftFootWaistX+rightFootWaistX)/2    
    
    waistDrop = self.initWaistH - self.waistXYZ[2]
    # print ("init height",format(self.initWaistH,".2f"), "current:", format(self.waistXYZ[2],".2f" ))
    # print ("DROP:", format(waistDrop,".2f" ))

    # print ("DEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD!!!") if abs(rollDeg) > 30 or pitchDeg > 45 or pitchDeg < -15 or waistDrop > 0.5 else +2
    return -1 if abs(rollDeg) > 30 or pitchDeg > 45 or pitchDeg < -15 else +2#or waistDrop > 0.5 else +2 

  def headXwaistX(self):
    headXWaistX = self.headP_XYZ[0] - self.waistXYZ[0]
    if headXWaistX<0: headXcost = (2 * np.exp( -300 * (headXWaistX/1)**2))-1
    else: headXcost = (2 * np.exp( -50 * (headXWaistX/2)**2))-1
    # print (format(headXWaistX,".2f"),"||", format(headXcost,".3f"))
    return (headXcost)

  def waistXYZfeetXY(self):
    leftFootWaistX = self.LAP_state[0][0] - self.waistXYZ[0]
    rightFootWaistX = self.RAP_state[0][0] - self.waistXYZ[0]
    self.avgFeetX = (leftFootWaistX+rightFootWaistX)/2
    if self.avgFeetX<0: waistFeetXCost = (2 * np.exp( -200 * (self.avgFeetX/2)**2))-1  # ; print ("feet BEHIND waist")
    else: waistFeetXCost = (2 * np.exp( -400 * (self.avgFeetX/8)**2))-1  # ; print ("feet IN FRONT of waist")
    # print ("average feet X", format(self.avgFeetX,".2f"))

    self.avgFeetY = (self.RAP_state[0][1] + self.LAP_state[0][1])
    waistLeftRight = (self.avgFeetY - self.waistXYZ[1])
    # if self.avgFeetY< 0: print("midFeet RIGHT")
    # if waistLeftRight==0: print("waist_Y MID")
    # if self.avgFeetY>0: print("midFeet LEFT")
    waistFeetYCost = (2 * np.exp( -70 * (waistLeftRight/2)**2))-1
    # print ("median feet Y", format(self.avgFeetY,".2f"), "waist L_R", format(waistLeftRight,".2f"))
    # print ("LEFT fOOt Y", format(self.LAP_state[0][1],".2f"), "RIGHT fOOt Y", format(self.RAP_state[0][1],".2f"))

    waistDrop2 = self.initWaistH - self.waistXYZ[2]
    if waistDrop2>0: waistZCost = (2 * np.exp( -75 * (waistDrop2/0.75)**2))-1
    else: waistZCost = (2 * np.exp( -1000 * (waistDrop2/0.99)**2))-1

    return (waistFeetXCost, waistFeetYCost, waistZCost)

  def headP_XYZfeetXY(self):
    leftFootHeadX = self.LAP_state[0][0] - self.headP_XYZ[0]
    rightFootHeadX = self.RAP_state[0][0] - self.headP_XYZ[0]
    self.avgFeetX = (leftFootHeadX+rightFootHeadX)/2
    if self.avgFeetX<0: headFeetXCost = (2 * np.exp( -200 * (self.avgFeetX/2)**2))-1  # ; print ("feet BEHIND waist")
    else: headFeetXCost = (2 * np.exp( -400 * (self.avgFeetX/8)**2))-1  # ; print ("feet IN FRONT of waist")
    # print ("average feet X", format(self.avgFeetX,".2f"))

    self.avgFeetY = (self.RAP_state[0][1] + self.LAP_state[0][1])
    headLeftRight = (self.avgFeetY - self.headP_XYZ[1])
    # if self.avgFeetY< 0: print("midFeet RIGHT")
    # if waistLeftRight==0: print("waist_Y MID")
    # if self.avgFeetY>0: print("midFeet LEFT")
    headFeetYCost = (2 * np.exp( -70 * (headLeftRight/2)**2))-1
    # print ("median feet Y", format(self.avgFeetY,".2f"), "waist L_R", format(waistLeftRight,".2f"))
    # print ("LEFT fOOt Y", format(self.LAP_state[0][1],".2f"), "RIGHT fOOt Y", format(self.RAP_state[0][1],".2f"))

    headDrop = self.initHeadH - self.headP_XYZ[2]
    if headDrop>0: headZCost = (2 * np.exp( -75 * (headDrop/0.75)**2))-1
    else: headZCost = (2 * np.exp( -1000 * (headDrop/0.99)**2))-1

    return (headFeetXCost, headFeetYCost, headZCost)

  def feetUnderAlive(self):
    waistDrop = self.initWaistH - self.waistXYZ[2]
    # print ("waist has DROPPED", format(waistDrop,".2f"))
    # waistDrop = self.initHeadH - self.headP_XYZ[2]

    # pybullet.addUserDebugText(str(self.t), [0,-0.8,1.8], textSize=3)

    waistBehindAheadFeet = False
    midFeetX = (self.LAR_state[0][0] + self.RAR_state[0][0])/2
    feetXwaistX = midFeetX - self.waistXYZ[0]
    frontFoot = max(self.LAR_state[0][0], self.RAR_state[0][0])
    backFoot = min(self.LAR_state[0][0], self.RAR_state[0][0])
    # if self.waistXYZ[0] < (backFoot-0.2) or 
    if self.waistXYZ[0] > (frontFoot+0.15) or self.waistXYZ[0] < (backFoot - 0.15): waistBehindAheadFeet=True
    # if abs(feetXwaistX) > 0.26: waistBehindAheadFeet=True   
    # print ("LEFT foot X", format(self.LAP_state[0][0],".2f"), "RIGHT foot X", format(self.RAP_state[0][0],".2f"), "WAIST X", format(self.waistXYZ[0],".2f"))

    waistOutsideFoot = False
    # leftFootWaistY = self.LAP_state[0][1] - self.waistXYZ[1]
    # rightFootWaistY = self.RAP_state[0][1] - self.waistXYZ[1]
    # if leftFootWaistY<0 or rightFootWaistY>0: waistOutsideFoot= True
    if self.waistXYZ[1] > self.LAR_state[0][1] or self.waistXYZ[1] < self.RAR_state[0][1]: waistOutsideFoot= True
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
    return -3 if waistDrop > 0.15 else 2#or waistBehindAheadFeet==True or waistOutsideFoot==True else +2 
    
  def dontMoveY(self):
    headYWaistY = self.headP_XYZ[1] - self.waistXYZ[1]
    headYcost = (2 * np.exp( -170 * (headYWaistY/3)**2))-1
    # print ((self.headP_XYZ[1]), headYcost)# and waistXcost))
    return (headYcost)
  
  def dontMoveZ(self):
    headZWaistZ = self.headP_XYZ[2] - self.waistXYZ[2]
    headZcost = (2 * np.exp( -170 * (headZWaistZ/3)**2))-1
    print (int(headZWaistZ), headZcost)
    return (headZcost)

  def stayUprightFn(self):
    headStraight, headTilt, chestStraight, chestTilt, waistStraight, waistTilt= 0,0,0,0,0,0
    waistDegress= list(self.waistAngleEuler) #(np.rad2deg(pybullet.getEulerFromQuaternion(self.waistState[1])))
    waistRoll, waistPitch = waistDegress[0], waistDegress[1]
    chestDegrees= list(np.rad2deg(pybullet.getEulerFromQuaternion(self.chestP_state[1])))
    chestRoll, chestPitch = chestDegrees[0], chestDegrees[1]   #CHESTrOLL VALUES are too LOW e.g. 1.0039241789312915e-16
    headDegrees= list(np.rad2deg(pybullet.getEulerFromQuaternion(pybullet.getLinkState(1,self.headP_id)[1])))
    headRoll, headPitch = headDegrees[0], headDegrees[1]
    if (np.abs(headPitch)>=20): headStraight= np.tanh(-1 * (np.abs(headPitch)/20))
    # if (np.abs(headRoll)>=20): headStraight= np.tanh(-1 * (np.abs(headRoll)/20))
    if (headPitch<20 and headPitch>-20 and headPitch!= 0): headStraight = np.tanh(+1 / (np.abs(headPitch)/20))
    elif(headPitch==0): headStraight=1         
    if (np.abs(headRoll)>=5): headTilt = np.tanh(-1 * (np.abs(headRoll)/5))
    if (np.abs(headRoll)<5 and headRoll!=0): headTilt = np.tanh(+1 / (np.abs(headRoll)/5))    
    elif(headRoll==0): headTilt=1         
    # print("HEAD:",(headPitch)) 

    if (chestPitch>=30): chestStraight = np.tanh(-1 * (np.abs(chestPitch)/30))
    if (chestPitch<=-20): chestStraight = np.tanh(-1 * (np.abs(chestPitch)/20))
    if (chestPitch<30 and chestPitch>-20 and chestPitch!= 0): chestStraight = np.tanh(+1 / (np.abs(chestPitch)/30))
    elif(chestPitch==0): chestStraight=1    
    if (np.abs(chestRoll)>=5): chestTilt = np.tanh(-1 * (np.abs(chestRoll)/5))
    if (np.abs(chestRoll)<5 and chestRoll!=0): chestTilt = np.tanh(+1 / (np.abs(chestRoll)/5)) 
    elif(chestRoll==0): chestTilt=1         
    # print("chest:",(chestPitch))    

    if (np.abs(waistPitch)>=10): waistStraight = np.tanh(-1 * (np.abs(waistPitch)/10))
    if (np.abs(waistPitch)<10 and waistPitch!=0): waistStraight = np.tanh(+1 / (np.abs(waistPitch)/10))
    elif(waistPitch==0): waistStraight=1
    if (np.abs(waistRoll)>=5): waistTilt = np.tanh(-1 * (np.abs(waistRoll)/5))
    if (np.abs(waistRoll)<5 and waistRoll!=0): waistTilt = np.tanh(+1 / (np.abs(waistRoll)/5))
    elif (waistRoll==0): waistTilt=1   # if (chestPitch<30 and chestPitch>-20): chestStraight = +0.1 / (np.abs(chestPitch)/30)
    # print("waist:", (waistTilt))
    # print("waistP:", (waistRoll))

    stayStraightReward=[headStraight, headTilt, chestTilt, chestStraight ,waistStraight ,waistTilt]
    return sum(stayStraightReward)

  def NFLrules(self):
    playerDown= False
    # for i in range(pybullet.getNumJoints(1)):
    # bodyFloor= set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, i))
    waistFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, -1))
    headFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.headP_id))
    chestFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.chestP_id))

    leftShinFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftKP))
    rightShinFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightKP))
    leftQuadFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftCP))
    rightQuadFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightCP))

    leftHandFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftH))
    leftWristFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftWY))
    leftElbowFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftEP))
    rightElbowFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightEP))
    rightForearmFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightEY))
    leftForearmFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftEY))
    rightShoulderFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightButLSP))
    leftShoulderFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.LButRSP))

    rightWristFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightWY))
    rightHandFloor = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightH))
    leftFootFloor2 = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.leftAP))
    rightFootFloor2 = set(x[0] for x in  pybullet.getContactPoints(0, 1, -1, self.rightAP))

    if (leftWristFloor or leftHandFloor or rightHandFloor or rightWristFloor or leftFootFloor2 or rightFootFloor2):
        # print("hand on floor, player UP")
        playerDown= False
    if (headFloor or waistFloor or chestFloor):
        # print ("player Dooooown - TORSO")
        playerDown= True      
    if (leftShinFloor or rightShinFloor or leftQuadFloor or rightQuadFloor ):
        # print ("player Dooooown - LEGS")
        playerDown= True
    if (leftElbowFloor or rightElbowFloor or leftForearmFloor or rightForearmFloor or leftShoulderFloor or rightShoulderFloor ):
        # print ("player Dooooown - ARMS")
        playerDown= True
    return +2 if playerDown== False else -1

  def flatFooted(self):
    leftFlat, rightFlat = 0, 0
    leftFootDeg= list(np.rad2deg(self.LAP_rad))
    rightFootDeg= list(np.rad2deg(self.RAP_rad))
    leftFootRoll, leftFootPitch= leftFootDeg[0], leftFootDeg[1]
    rightFootRoll, rightFootPitch= rightFootDeg[0], rightFootDeg[1]
    # print("left foot:",(leftFootDeg)) 
    # print("left roll:",(leftFootRoll)) 
    leftNoRoll= (2 * np.exp( -0.1 * (leftFootRoll/2)**2))-1.0
    rightNoRoll= (2 * np.exp( -0.1 * (rightFootRoll/2)**2))-1.0
    leftNoPitch= (2 * np.exp( -0.1 * (leftFootPitch/5)**2))-1.0
    rightNoPitch= (2 * np.exp( -0.1 * (rightFootPitch/5)**2))-1.0

    # if (np.abs(leftFootPitch)>=10): leftFlat = np.tanh(-1 * (np.abs(leftFootPitch)/20))
    # if (np.abs(leftFootPitch)<10 and leftFootPitch!=0): leftFlat = np.tanh(+1 / (np.abs(leftFootPitch)/5))    
    # if (np.abs(rightFootPitch)>=10): rightFlat = np.tanh(-1 * (np.abs(rightFootPitch)/20))
    # if (np.abs(rightFootPitch)<10 and rightFootPitch!=0): rightFlat = np.tanh(+1 / (np.abs(rightFootPitch)/5))      

    # if (rightFootRoll<-20): rightFlat = np.tanh(-1 * (np.abs(rightFootRoll)/40)) #20/40 from +0.2 to -0.5 at -20deg
    # if (rightFootRoll>0): rightFlat = np.tanh(-1 * np.abs(rightFootRoll)) #do not bend inwards
    # if (rightFootRoll<0 and rightFootRoll>=-20): rightFlat = np.tanh(+1 / (np.abs(rightFootRoll)/5))    
    # if (leftFootRoll>20): leftFlat = np.tanh(-1 * (np.abs(leftFootRoll)/40))
    # if (leftFootRoll<0): leftFlat = np.tanh(-1 * np.abs(leftFootRoll))
    # if (leftFootRoll>0 and leftFootRoll<=20): leftFlat = np.tanh(+1 / (np.abs(leftFootRoll)/5))    
    
    # flatFeetReward= [leftFlat, rightFlat]
    leftFootRP= (leftNoRoll+ leftNoPitch)/2
    rightFootRP= (rightNoRoll+ rightNoPitch)/2
    return leftFootRP, rightFootRP

  def feet_ground_contact(self):
    rightFootFloor = set(x[0] for x in  pybullet.getContactPoints(1, 0, self.rightAP, -1)) # L_ANKLE_P_LINK
    leftFootFLoor = set(x[0] for x in  pybullet.getContactPoints(1, 0, self.leftAP, -1)) # R_ANKLE_P_LINK 
    leftFootOnGround, rightFootOnGround = 0.0, 0.0
    if (leftFootFLoor): leftFootOnGround = 0.5 #; print ("LEFT foot on floor:")#, self.foot_on_ground)      
    if (rightFootFloor): rightFootOnGround= 0.5 #; print ("RIGHT foot on floor:")#, self.foot_on_ground)
    self.feet_on_ground= np.array([leftFootOnGround, rightFootOnGround])
    return -1 if sum(self.feet_on_ground) ==0 else sum(self.feet_on_ground)

  def torsoHeights(self):
    waistHigh, chestHigh, headHigh= 0,0,0
    waistH, chestH, headH, kneeH= self.waistXYZ[2], self.chestP_XYZ[2], self.headP_XYZ[2], self.LKP_state[0][2]
    badPose=False
    if (headH <= chestH): headHigh=-1; badPose=True
    if (chestH <= waistH): chestHigh=-1; badPose=True
    if (waistH <= kneeH): waistHigh=-1; badPose=True
    # if (badPose==True):  waistHigh, chestHigh, headHigh= -1, -1, -1
    if (badPose==False):
      print('bad Pose is False')

    # print("head height:", headH, "; chest height:", chestH, "; waist height", waistH)
    # print("initial head:", self.initHeadH, "; initial chest:", self.initChestPH, "; initial waist", self.initWaistH)
    # print ("knee init:", self.LKP_state[0][2], "knee+now:", pybullet.getLinkState(1,self.leftKP)[0][2])
    # if (waistH==self.initWaistH or (waistH>=(self.initWaistH-0.15))): waistHigh=( +1 * (waistH/self.initWaistH))
    # if (waistH<(self.initWaistH-0.15)): waistHigh=np.tanh(-1 / (10*waistH/self.initWaistH))
    # if (waistH> self.initWaistH or waistH<=kneeH): waistHigh=-1

    # if (chestH<(self.initChestPH-0.2)): chestHigh= np.tanh(-1 / (3*chestH/self.initChestPH))
    # if (chestH<self.initChestPH-0.2 and chestH>waistH): chestHigh= np.tanh(-1 / (20*chestH/self.initChestPH))
    # if (chestH>waistH and waistH>kneeH): chestHigh= 0.1*chestH/waistH
    # if (chestH <= waistH or chestH> self.initChestPH): chestHigh=-1
    # if (chestH==self.initChestPH or (chestH > waistH and chestH>=(self.initChestPH-0.2) and chestH<self.initChestPH)): chestHigh= (chestH/self.initChestPH)
    stayingHighReward = [waistHigh, chestHigh, headHigh]
    return sum(stayingHighReward)

  def keepLegsInitial(self):
    currentList=[]
    for i in  range(pybullet.getNumJoints(1))[20:]:
      current= pybullet.getJointState(1, i)[0]
      currentList.append(current)
    print ((currentList))

  def longLife(self, LLMultiplier):
    liveLong = LLMultiplier * (self.no_of_steps / 1500)    
    # np.clip(liveLong, 0, 2.5)
    # longLife = np.tanh(liveLong)
    return np.clip(liveLong, 0, 2.5)

  def alive_bonus(self, alive_z):#, pitch):
    # print('alive bonus Z = ', alive_z)
    # return +2 if alive_z > 0.9 else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying
    return +2 if alive_z >= self.LCP_state[0][2] else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying


class KClass(WalkerBaseURDF):
  # self_collision = False #works without so ...
  # quad_list = ["R_CROTCH_P_LINK", "L_CROTCH_P_LINK"]  # right quad, left quad
  foot_list = ["R_ANKLE_P_LINK", "L_ANKLE_P_LINK"]
  # foot_list += ["R_KNEE_P_LINK", "L_KNEE_P_LINK", "R_ANKLE_P_LINK", "L_ANKLE_P_LINK"]  # right SHIN, left SHIN
  def __init__(self):
    WalkerBaseURDF.__init__(self,
    '/home/admin/dribble_repo/RHP5E_anklez.urdf', #mod2.urdf',
    # '/home/admin/dribble_repo/RHP5E_gundam.urdf',
    # '/home/admin/dribble_repo/RHP5E_chappie.urdf',
    # '/home/jovyan/private/drb/RHP5E_late_jup.urdf',
    'robot_name_WBURDF',
    action_dim= 30,  #no of joints to play with
    obs_dim= 246, # ???
    power= 0.3) #because 30% EFFORT ????
    # 17 joints, 4 of them important for walking (hip, knee), others may as well be turned off, 17/4 = 4.25
    # 32 / 13 important standing joints = 2.461538462
    self.choreonoidPose= [
      0, #0 CHEST_Y
      0, #1 CHEST_P
      0, #2 HEAD_Y
      0, #3 HEAD_P

      0.2618,  #4 L_SHOULDER_P
      0.2618, #5 L_SHOULDER_R #flips -/+ by itself (originally) + !! + again for apply, but - in initial
      -0.0873, #6 L_SHOULDER_Y
      -0.5236, #7 L_ELBOW_P
      0, #8 L_ELBOW_Y
      0, #9 L_WRIST_P
      0, #10 L_WRIST_Y
      0, #-0.5061, #11 LH_state

      0.2618, #12 R_SHOULDER_P
      -0.2618, #13 R_SHOULDER_R #flips -/+ by itself (originally) -
      0.0873, #14 R_SHOULDER_Y
      -0.5236, #15 R_ELBOW_P
      0, #16 R_ELBOW_Y
      0, #17 R_WRIST_P
      0, #18 R_WRIST_Y
      0, #-0.5061, #19 RH_state

      0, #20 L_CROTCH_Y
      0.1745, #21 L_CROTCH_R
      -0.2967, #22 L_CROTCH_P
      0.6283, #23 L_KNEE_P
      -0.1745, #24 L_ANKLE_R
      -0.2967, #25 L_ANKLE_P

      0, #26 R_CROTCH_Y
      -0.1745, #27 R_CROTCH_R
      -0.2967, #28 R_CROTCH_P
      0.6283, #29 R_KNEE_P
      0.1745, #30 R_ANKLE_R
      -0.2967 #31 R_ANKLE_P
    ]
  def robot_specific_reset(self, bullet_client):
    WalkerBaseURDF.robot_specific_reset(self, bullet_client)
    self.motor_names  = ["CHEST_Y"];  self.motor_power = [347.5636]; self.pGainList = [44000]; self.vGainList = [400]; self.maxVelList = [366.667]
    self.motor_names += ["CHEST_P"]; self.motor_power += [347.5636]; self.pGainList+= [44000]; self.vGainList+= [440]; self.maxVelList+= [366.667]
    self.motor_names += ["HEAD_Y"];  self.motor_power += [22.2]; self.pGainList+= [2000]; self.vGainList+= [50]; self.maxVelList+= [1800] 
    self.motor_names += ["HEAD_P"];  self.motor_power += [22.2]; self.pGainList+= [2000]; self.vGainList+= [50]; self.maxVelList+= [1800]
    # self.motor_names += ["LC-W-base-joint", "LC-W-linear-joint"]; self.motor_power += [0.0, 0.0]    
    self.motor_names += ["L_SHOULDER_P"]; self.motor_power += [178.42]; self.pGainList+= [15000]; self.vGainList+= [240]; self.maxVelList+= [357.143]  
    self.motor_names += ["L_SHOULDER_R"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714]
    self.motor_names += ["L_SHOULDER_Y"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714] 
    # self.motor_names += ["LC-E-base-joint", "LC-E-linear-joint"]; self.motor_power += [0.0, 0.0]
    self.motor_names += ["L_ELBOW_P"]; self.motor_power += [247.16]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.813]
    self.motor_names += ["L_ELBOW_Y"]; self.motor_power += [123.62]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.727]    
    self.motor_names += ["L_WRIST_P"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400] 
    self.motor_names += ["L_WRIST_Y"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]  
    self.motor_names += ["L_HAND"];    self.motor_power += [94.4];   self.pGainList+= [1];     self.vGainList+= [0.1]; self.maxVelList+= [675]       
    # self.motor_names += ["RC-W-base-joint", "RC-W-linear-joint"]; self.motor_power += [0.0, 0.0]
    self.motor_names += ["R_SHOULDER_P"]; self.motor_power += [178.42]; self.pGainList+= [15000]; self.vGainList+= [240]; self.maxVelList+= [357.143]  
    self.motor_names += ["R_SHOULDER_R"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714]
    self.motor_names += ["R_SHOULDER_Y"]; self.motor_power += [185.85]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [685.714] 
    # self.motor_names += ["RC-E-base-joint", "RC-E-linear-joint"]; self.motor_power += [0.0, 0.0]    
    self.motor_names += ["R_ELBOW_P"]; self.motor_power += [247.16]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.813]
    self.motor_names += ["R_ELBOW_Y"]; self.motor_power += [123.62]; self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [257.727]     
    self.motor_names += ["R_WRIST_P"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]
    self.motor_names += ["R_WRIST_Y"]; self.motor_power += [159.3];  self.pGainList+= [14000]; self.vGainList+= [240]; self.maxVelList+= [400]
    self.motor_names += ["R_HAND"];    self.motor_power += [94.4];   self.pGainList+= [1];     self.vGainList+= [0.1]; self.maxVelList+= [675]
   
    self.motor_names += ["L_CROTCH_Y"]; self.motor_power += [112.38];   self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [283.5]
    self.motor_names += ["L_CROTCH_R"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]
    self.motor_names += ["L_CROTCH_P"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]   
    # self.motor_names += ["LC-CI-base-joint", "LC-CI-linear-joint", "LC-CI-hinge-joint", "LC-CO-base-joint" "LC-CO-linear-joint", "LC-CO-hinge-joint", "LC-K-base-joint", "LC-K-linear-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0, 0, 0]    
    self.motor_names += ["L_KNEE_P"];   self.motor_power += [477.9]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [200]             
    # self.motor_names += ["LC-AI-base-joint", "LC-AI-linear-joint", "LC-AI-hinge-joint", "LC-AO-base-joint" "LC-AO-linear-joint", "LC-AO-hinge-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0]     
    self.motor_names += ["L_ANKLE_R"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667]     
    # self.motor_names += ["L_ANKLE_P"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667] 
    
    self.motor_names += ["R_CROTCH_Y"]; self.motor_power += [112.38];   self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [283.5]
    self.motor_names += ["R_CROTCH_R"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]
    self.motor_names += ["R_CROTCH_P"]; self.motor_power += [347.5636]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [366.667]   
    # self.motor_names += ["RC-CI-base-joint", "RC-CI-linear-joint", "RC-CI-hinge-joint", "RC-CO-base-joint" "RC-CO-linear-joint", "RC-CO-hinge-joint", "RC-K-base-joint", "RC-K-linear-joint"] ; self.motor_power += [0, 0, 0, 0, 0, 0, 0, 0]    
    self.motor_names += ["R_KNEE_P"];   self.motor_power += [477.9]; self.pGainList+= [20000]; self.vGainList+= [400]; self.maxVelList+= [200]             
    # self.motor_names += ["RC-AI-base-joint", "RC-AI-linear-joint", "RC-AI-hinge-joint", "RC-AO-base-joint" "RC-AO-linear-joint", "RC-AO-hinge-joint"]; self.motor_power += [0, 0, 0, 0, 0, 0]              
    self.motor_names += ["R_ANKLE_R"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667]    
    # self.motor_names += ["R_ANKLE_P"];  self.motor_power += [347.5636]; self.pGainList+= [10000]; self.vGainList+= [300]; self.maxVelList+= [366.667] 

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
    # assert (np.isfinite(action).all()); assert (len(self.motor_list) == len(action))
    # print ("acion function", action)
    # f = open('/home/admin/dribble_repo/action_log.txt','a') # Prints Action Values
    for jointIndex, motorListIndex, powerIndex, pgain, vgain, velIndex, nameIndex in zip(range(len(self.motor_list)), self.motor_list, self.motor_power, self.pGainList, self.vGainList, self.maxVelList, self.motor_names):
      lo3, hi3= pybullet.getJointInfo(1,jointIndex)[8], pybullet.getJointInfo(1, jointIndex)[9]
      self.desired_position= np.clip(action[jointIndex], lo3, hi3)
      desired_torque= self.power*powerIndex
      maxVel= np.deg2rad(velIndex)


      # if jointIndex>19: motorListIndex.set_position(initialPoseList2[jointIndex], desired_torque, maxVel)

      # if i==self.leftbutRightSR: motorListIndex.set_position(np.clip(action[i], lo3, 0), desired_torque, maxVel) #ACTUALLy RIGHT 
      # if i==self.rightButLeftSR: motorListIndex.set_position(np.clip(action[i], 0, hi3), desired_torque, maxVel) #ACTUALLY LEFT ARM
      
      # if jointIndex==self.leftAP or jointIndex==self.rightAP: motorListIndex.set_position(np.clip(action[jointIndex], 0, hi3), desired_torque, maxVel) #ACTUALLY LEFT ARM
      # if jointIndex>19: motorListIndex.set_position(np.clip(action[jointIndex], 0, hi3), desired_torque, maxVel) #ACTUALLY LEFT ARM

      # else:
      motorListIndex.set_position(self.desired_position, desired_torque, maxVel)
      self.choreonoidPose[jointIndex] = self.desired_position
      # return action
      # motorListIndex.set_position(desired_position, desired_torque, maxVel)
      # motorListIndex.set_positionNoVel(desired_position, desired_torque)
      # motorListIndex.positionPD(desired_position, desired_torque, maxVel, pgain, vgain)
      # motorListIndex.pdControl(desired_position, desired_torque, pgain, vgain, maxVel)
      # f.write(str(desired_position)); f.write(" ")
      # self._p.setJointMotorControl2(1, jointIndex, pybullet.POSITION_CONTROL, targetPosition= desired_position, force=desired_torque, maxVelocity=maxVel)#, positionGain=pgain, velocityGain=vgain)
      # whatjoint = self.leftCP
      # print("joint: "+ self.motor_names[whatjoint])
      # print("a: ", action[whatjoint])
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
      goal.trajectory.joint_names.extend(["CHEST_Y", "CHEST_P"]) #0 #1
      goal.trajectory.joint_names.extend(["HEAD_Y", "HEAD_P"]) #2 #3
      goal.trajectory.joint_names.extend(["L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y"]) #4 #5 #6
      goal.trajectory.joint_names.extend(["L_ELBOW_P", "L_ELBOW_Y"])#7 #8
      goal.trajectory.joint_names.extend(["L_WRIST_P", "L_WRIST_Y", "LH_state"])#9 #10 #11

      goal.trajectory.joint_names.extend(["R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y"]) #12 #13 #14
      goal.trajectory.joint_names.extend(["R_ELBOW_P", "R_ELBOW_Y"])#15 #16
      goal.trajectory.joint_names.extend(["R_WRIST_P", "R_WRIST_Y", "RH_state"]) #17 #18 #19

      goal.trajectory.joint_names.extend(["L_CROTCH_Y", "L_CROTCH_R", "L_CROTCH_P"]) #20 #21 #22
      goal.trajectory.joint_names.extend(["L_KNEE_P", "L_ANKLE_R", "L_ANKLE_P"]) #23 #24 #25
      goal.trajectory.joint_names.extend(["R_CROTCH_Y", "R_CROTCH_R", "R_CROTCH_P"]) #26 #27 #28
      goal.trajectory.joint_names.extend(["R_KNEE_P", "R_ANKLE_R", "R_ANKLE_P"]) #29 #30 #31

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

def get_cube(_p, x, y, z):
  body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(), "cube_small.urdf"), [x, y, z])
  _p.changeDynamics(body, -1, mass=1.2)  #match Roboschool
  link_name, _ = _p.getBodyInfo(body)
  link_name = link_name.decode("utf8")
  bodies = [body]
  return BodyPart(_p, link_name, bodies, 0, -1)

def get_sphere(_p, x, y, z):
  body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(), "sphere2red_nocol.urdf"), [x, y, z])
  link_name, _ = _p.getBodyInfo(body)
  link_name = link_name.decode("utf8")
  bodies = [body]
  return BodyPart(_p, link_name, bodies, 0, -1)

class KFlagrunClass(KClass):

  def __init__(self):
    KClass.__init__(self)
    self.flag = None

  def robot_specific_reset(self, bullet_client):
    KClass.robot_specific_reset(self, bullet_client)
    self.flag_reposition()

  def flag_reposition(self):
    self.walk_target_x = self.np_random.uniform(low=-self.scene.stadium_halflen,
                                                high=+self.scene.stadium_halflen)
    self.walk_target_y = self.np_random.uniform(low=-self.scene.stadium_halfwidth,
                                                high=+self.scene.stadium_halfwidth)
    more_compact = 0.5  # set to 1.0 whole football field
    self.walk_target_x *= more_compact
    self.walk_target_y *= more_compact

    if (self.flag):
      #for b in self.flag.bodies:
      #	print("remove body uid",b)
      #	pitch.removeBody(b)
      pybullet.resetBasePositionAndOrientation(self.flag.bodies[0],
                                              [self.walk_target_x, self.walk_target_y, 0.7],
                                              [0, 0, 0, 1])
    else:
      self.flag = get_sphere(pybullet, self.walk_target_x, self.walk_target_y, 0.7)
    self.flag_no_of_steps = 600 / self.scene.frame_skip  #match Roboschool

  def calc_state(self):
    self.flag_no_of_steps -= 1
    state = KClass.calc_state(self)
    if self.walk_target_dist < 1 or self.flag_no_of_steps <= 0:
      self.flag_reposition()
      state = KClass.calc_state(self)  # caclulate state again, against new flag pos
      self.potential = self.calc_potential()  # avoid reward jump
    return state

class KFlagrunHarderClass(KClass):

  def __init__(self):
    KFlagrunClass.__init__(self)
    self.flag = None
    self.aggressive_cube = None
    self.frame = 0

  def robot_specific_reset(self, bullet_client):

    KFlagrunClass.robot_specific_reset(self, bullet_client)

    self.frame = 0
    if (self.aggressive_cube):
      pybullet.resetBasePositionAndOrientation(self.aggressive_cube.bodies[0], [-1.5, 0, 0.05],
                                              [0, 0, 0, 1])
    else:
      self.aggressive_cube = get_cube(pybullet, -1.5, 0, 0.05)
    self.on_ground_frame_counter = 0
    self.crawl_start_potential = None
    self.crawl_ignored_potential = 0.0
    self.initial_z = 0.8

  def alive_bonus(self, z, pitch):
    if self.frame % 30 == 0 and self.frame > 100 and self.on_ground_frame_counter == 0:
      target_xyz = np.array(self.waistXYZ)
      robot_speed = np.array(self.robot_body.speed())
      angle = self.np_random.uniform(low=-3.14, high=3.14)
      from_dist = 4.0
      attack_speed = self.np_random.uniform(
          low=20.0, high=30.0)  # speed 20..30 (* mass in cube.urdf = impulse)
      time_to_travel = from_dist / attack_speed
      target_xyz += robot_speed * time_to_travel  # predict future position at the moment the cube hits the robot
      position = [
          target_xyz[0] + from_dist * np.cos(angle), target_xyz[1] + from_dist * np.sin(angle),
          target_xyz[2] + 1.0
      ]
      attack_speed_vector = target_xyz - np.array(position)
      attack_speed_vector *= attack_speed / np.linalg.norm(attack_speed_vector)
      attack_speed_vector += self.np_random.uniform(low=-1.0, high=+1.0, size=(3,))
      self.aggressive_cube.reset_position(position)
      self.aggressive_cube.reset_velocity(linearVelocity=attack_speed_vector)
    if z < 0.8:
      self.on_ground_frame_counter += 1
    elif self.on_ground_frame_counter > 0:
      self.on_ground_frame_counter -= 1
    # End episode if the robot can't get up in 170 frames, to save computation and decorrelate observations.
    self.frame += 1
    return self.potential_leak() if self.on_ground_frame_counter < 170 else -1

  def potential_leak(self):
    z = self.waistXYZ[2]  # 0.00 .. 0.8 .. 1.05 normal walk, 1.2 when jumping
    z = np.clip(z, 0, 0.8)
    return z / 0.8 + 1.0  # 1.00 .. 2.0

  def calc_potential(self):
    # We see alive bonus here as a leak from potential field. Value V(s) of a given state equals
    # potential, if it is topped up with gamma*potential every frame. Gamma is assumed 0.99.
    #
    # 2.0 alive bonus if z>0.8, potential is 200, leak gamma=0.99, (1-0.99)*200==2.0
    # 1.0 alive bonus on the ground z==0, potential is 100, leak (1-0.99)*100==1.0
    #
    # Why robot whould stand up: to receive 100 points in potential field difference.
    flag_running_progress = KClass.calc_potential(self)

    # This disables crawl.
    if self.waistXYZ[2] < 0.8:
      if self.crawl_start_potential is None:
        self.crawl_start_potential = flag_running_progress - self.crawl_ignored_potential
        #print("CRAWL START %+0.1f %+0.1f" % (self.crawl_start_potential, flag_running_progress))
      self.crawl_ignored_potential = flag_running_progress - self.crawl_start_potential
      flag_running_progress = self.crawl_start_potential
    else:
      #print("CRAWL STOP %+0.1f %+0.1f" % (self.crawl_ignored_potential, flag_running_progress))
      flag_running_progress -= self.crawl_ignored_potential
      self.crawl_start_potential = None

    return flag_running_progress + self.potential_leak() * 100
