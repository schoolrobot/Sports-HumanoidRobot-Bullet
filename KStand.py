#44  default friction and physics

from drb_robot_bases import URDFBasedRobot#, BodyPart
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
    floorFriction= 0.9
    # pybullet.changeDynamics(0, -1, mass=0 
    #                         # ,lateralFriction= floorFriction, rollingFriction=floorFriction, spinningFriction=floorFriction 
    #                         # ,linearDamping=1, angularDamping=1 
    #                         # ,contactDamping=-1 ,contactStiffness=-1
    #                         ,restitution= 1
    #                         # ,frictionAnchor=1
    #                         )
    pybullet.enableJointForceTorqueSensor(1, self.leftAP, enableSensor= 1)
    pybullet.enableJointForceTorqueSensor(1, self.rightAP, enableSensor= 1)
    # pybullet.enableJointForceTorqueSensor(1, self.rightWP, enableSensor= 1)
    # pybullet.enableJointForceTorqueSensor(1, self.leftWP, enableSensor= 1)

  def robot_specific_reset(self, bullet_client):
    pybullet = bullet_client
    self.t = 0

    self.no_of_steps= 0

    KHI_Walk_poseList= [
      0, #0 CHEST_Y
      0, #1 CHEST_P
      0, #2 HEAD_Y
      0, #3 HEAD_P

      0.2618,  #4 L_SHOULDER_P
      -0.0873, #5 L_SHOULDER_R #flips -/+ by itself (originally) +
      -0.0873, #6 L_SHOULDER_Y
      -0.5236, #7 L_ELBOW_P
      0, #8 L_ELBOW_Y
      0, #9 L_WRIST_P
      0, #10 L_WRIST_Y
      0.8901, #-0.5061, #11 LH_state

      0.2618, #12 R_SHOULDER_P
      0.0873, #13 R_SHOULDER_R #flips -/+ by itself (originally) -
      0.0873, #14 R_SHOULDER_Y
      -0.5236, #15 R_ELBOW_P
      0, #16 R_ELBOW_Y
      0, #17 R_WRIST_P
      0, #18 R_WRIST_Y
      0.8901, #-0.5061, #19 RH_state

      0, #20 L_CROTCH_Y
      0, #21 L_CROTCH_R
      -0.2967, #22 L_CROTCH_P
      0.6283, #23 L_KNEE_P
      0, #24 L_ANKLE_R
      -0.3316, #25 L_ANKLE_P

      0, #26 R_CROTCH_Y
      0, #27 R_CROTCH_R
      -0.2967, #28 R_CROTCH_P
      0.6283, #29 R_KNEE_P
      0, #30 R_ANKLE_R
      -0.3316 #31 R_ANKLE_P
    ]

    KHI_Walk_poseList_14= [
      0.2618, #0 R_SHOULDER_P pybullet LEFT
      -0.0873, #1 R_SHOULDER_R pyBULLET LEFT
      -0.0873, #2 R_SHOULDER_Y pyBULLET LEFT
      -0.5236, #3 R_ELBOW_P
      
      -0.2967, #4 R_CROTCH_P
      0.6283, #5 R_KNEE_P
      -0.3316, #6 R_ANKLE_P      
      
      -0.2967, #7 L_CROTCH_P
      0.6283, #8 L_KNEE_P
      -0.3316, #9 L_ANKLE_P

      0.2618,  #10 L_SHOULDER_P pybullet RIGHT
      0.0873, #11 L_SHOULDER_R choreonoid negative
      0.0873, #12 L_SHOULDER_Y
      -0.5236 #13 L_ELBOW_P
    ]

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
    for i, j in zip(range(0,14),self.ordered_joints): #zip range because joint type not iterable
    # for i in range(pybullet.getNumJoints(1)):
      lo, hi= pybullet.getJointInfo(1,i)[8], pybullet.getJointInfo(1, i)[9]
      # footFriction=0.9; footDamping=1; damping= 3
      # if i==self.leftAP or i==self.rightAP: pybullet.changeDynamics(1, i
      #                                       ,lateralFriction=footFriction, rollingFriction=footFriction, spinningFriction=footFriction
      #                                       ,linearDamping=footDamping, angularDamping=footDamping
      #                                       ,contactDamping=-1 ,contactStiffness=-1 
      #                                       ,restitution=0
      #                                       ,frictionAnchor=1 ,contactProcessingThreshold=1
      #                                       )
      # else: pybullet.changeDynamics(1, i, lateralFriction=1, rollingFriction=1, spinningFriction=1,
      #                               linearDamping=1, angularDamping=1, jointDamping=1, restitution=0)
      j.disable_motor()
      # j.reset_current_position(0, 0)
      j.reset_current_position(KHI_Walk_poseList_14[i], 0)
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
    # self.initHeadH= pybullet.getLinkState(1,self.headP_id)[0][2]
    # self.initChestPH= pybullet.getLinkState(1,self.chestP_id)[0][2]
    # self.initChestY= pybullet.getLinkState(1,self.chestY_id)
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

  def bodyPartHeights(self):
    headAboveChestP, headAbovewAIST, chestAboveWaist,crotchPAboveKnee, WaistAboveKnee, WaistAboveCrotch, waistKneesAligned, kneeAboveFoot= 0,0,0,0,0,0,0,0
    botchCrotchPPs= [LCP_state[0][2], RCP_state[0][2]]
    bothKneeHeights = [LKP_state[0][2], RKP_state[0][2]]
    bothFeetHeights= [self.LAP_state[0][2], self.RAP_state[0][2]]
    # if self.headP_XYZ[2] < self.chestP_XYZ[2]: headAboveChestP= -1  ;print("HEAD < below  CHEST P")
    if self.headP_XYZ[2] <= self.waistXYZ[2]: headAbovewAIST= -1  ;print("HEAD < below  waist")
    # if self.chestP_XYZ[2] < self.waistXYZ[2]: chestAboveWaist= -1   ;print("CHEST below WAIST")
    if self.waistXYZ[2] <= min(botchCrotchPPs): WaistAboveCrotch= -1   ;print("WAIST below CROTCH P")
    # if self.waistXYZ[2] < max(bothKneeHeights): WaistAboveKnee= -1   ;print("WAIST below KNEEs")
    if min(botchCrotchPPs) < min(bothKneeHeights): crotchPAboveKnee= -1   ;print("CROTCH PPPP below KNEEE")
    if min(bothKneeHeights) < min(bothFeetHeights): kneeAboveFoot= -1   ;print("knEEs below FEET")
   # if (self.waistXYZ[0]> max([self.LAP_state[0][0], self.RAP_state[0][0]])): WaistAboveKnee= -1   ;print("WAIST behind/front of Feet X")
   # if LKP_state[0][2] > self.initLKnee : X= -1 ; print("KNEES dropped")
    # if (self.waistXYZ[0] - self.headP_XYZ[0]) > 0.15: waistKneesAligned= -1 ; print("Head/Waist NOT ALIGNED")
    bodyPartHeightReward= [headAbovewAIST, WaistAboveCrotch, crotchPAboveKnee, kneeAboveFoot]
    # print("left knee now", float(LKP_state[0][2]), "left knee init", float(self.initLKnee))
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
    waistDegress= list(self.waistAngleEuler) #(np.rad2deg(pybullet.getEulerFromQuaternion(waistState[1])))
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
    if (leftFootFLoor): leftFootOnGround = 1 #; print ("LEFT foot on floor:")#, self.foot_on_ground)      
    if (rightFootFloor): rightFootOnGround= 1 #; print ("RIGHT foot on floor:")#, self.foot_on_ground)
    self.feet_on_ground= np.array([leftFootOnGround, rightFootOnGround])
    return -1 if sum(self.feet_on_ground) ==0 else sum(self.feet_on_ground)

  def torsoHeights(self):
    waistHigh, chestHigh, headHigh= 0,0,0
    waistH, chestH, headH, kneeH= self.waistXYZ[2], self.chestP_XYZ[2], self.headP_XYZ[2], LKP_state[0][2]
    badPose=False
    if (headH <= chestH): headHigh=-1; badPose=True
    if (chestH <= waistH): chestHigh=-1; badPose=True
    if (waistH <= kneeH): waistHigh=-1; badPose=True
    # if (badPose==True):  waistHigh, chestHigh, headHigh= -1, -1, -1
    if (badPose==False):
      print('bad Pose is False')

    # print("head height:", headH, "; chest height:", chestH, "; waist height", waistH)
    # print("initial head:", self.initHeadH, "; initial chest:", self.initChestPH, "; initial waist", self.initWaistH)
    # print ("knee init:", LKP_state[0][2], "knee+now:", pybullet.getLinkState(1,self.leftKP)[0][2])
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

  def longLife(self, LLMultiplier):
    liveLong = LLMultiplier * (self.no_of_steps / 1500)    
    # np.clip(liveLong, 0, 2.5)
    # longLife = np.tanh(liveLong)
    return np.clip(liveLong, 0, 2.5)

  def alive_bonus(self, alive_z):#, pitch):
    # print('alive bonus Z = ', alive_z)
    # return +2 if alive_z > 0.9 else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying
    return +2 if alive_z >= LCP_state[0][2] else -1  # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying

  def waistEulerAlive(self):
    reward =10
    rollDeg, pitchDeg = np.rad2deg(self.waistAngleEuler[0]), np.rad2deg(self.waistAngleEuler[1])
    waistDrop = self.initWaistH - self.waistXYZ[2]
    # print ("PITCH: ", format(pitchDeg, ".2f"), "H: ", format(waistDrop, ".2f"), "ROLL: ", format(rollDeg, ".2f"))
    if abs(pitchDeg) <=30 and abs(pitchDeg) > 25: reward =0
    if abs(pitchDeg) <=25 and abs(pitchDeg) > 20: reward =1
    if abs(pitchDeg) <=20 and abs(pitchDeg) > 15: reward =2
    if abs(pitchDeg) <=15 and abs(pitchDeg) > 10: reward =3
    if abs(pitchDeg) <=10 and abs(pitchDeg) >  5: reward =4
    if abs(pitchDeg) <= 5: reward =5
    if abs(pitchDeg) > 30 or waistDrop > 0.10: reward = -5
    return reward
    # return -3 if abs(rollDeg) > 15 or abs(pitchDeg) > 30 or waistDrop > 0.15 else +3 

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

    # waistVel= pybullet.getBaseVelocity(1)
    # print("BASE VELOCITY |", "X:", format(waistVel[0][0],".2f"), "  Y:", format(waistVel[0][1],".2f"), "  Z:", format(waistVel[0][2],".2f"))
    # print("LEFT FOOT VELOCITY |", "X:", format(self.LAP_state[6][0],".2f"), "  Y:", format(self.LAP_state[6][1],".2f"), "  Z:", format(self.LAP_state[6][2],".2f"))
    # print ("LEFT_AP_STATE:", self.LAP_state[6])

    # more = np.array(
    #     [ z - self.initial_z,
    #       # np.sin(angle_to_target), np.cos(angle_to_target),
    #       # 0.3 * vx, 0.3 * vy, 0.3 * vz,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
    #       # roll, pitch
    #     ], dtype=np.float32)
    
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

          + [LSP_XYZ_rel] + [RSP_XYZ_rel]  #left arm XYZ
          # + [LEP_XYZ_rel] + [REP_XYZ_rel]  #right arm XYZ
          # + [self.waistXYZ] 

          + [LCP_XYZ_rel] + [RCP_XYZ_rel] 
          + [LKP_XYZ_rel] + [RKP_XYZ_rel] 
          + [LAP_XYZ_rel] + [RAP_XYZ_rel] 

          # + [LSP_orient] + [RSP_orient] #shoulders
          # + [LEP_orient] + [REP_orient] #elbows

          # + [LCP_orient] + [RCP_orient] #Crotch RPY
          # + [LKP_orient] + [RKP_orient] #Knees RPY
          # + [LAP_orient] + [RAP_orient] #FEET ORIENT = 6
          
          + [WAIST_ORIENT] #6

          + [np.tanh(np.divide(LAP_force,1000))] + [np.tanh(np.divide(RAP_force,1000))]
          + [np.tanh(np.divide(LAP_MoM,1000))] + [np.tanh(np.divide(RAP_MoM,1000))] #6
          # + [self.feet_on_ground] 
          )
    # return np.clip(np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

class KClass(WalkerBaseURDF):
  # self_collision = False #works without so ...
  foot_list = ["R_ANKLE_P_LINK", "L_ANKLE_P_LINK"]
  def __init__(self): WalkerBaseURDF.__init__(self,
    # '/home/admin/dribble_repo/RHP5E_14.urdf', #mod2.urdf',
    # '/home/admin/dribble_repo/RHP5E_gundam.urdf',
    # '/home/admin/dribble_repo/RHP5E_chappie.urdf',
    '/home/jovyan/private/drb/urdfs/RHP5E_14.urdf',
    'robot_name_WBURDF',
    action_dim= 14,  #no of joints to play with
    obs_dim= 61, 
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
