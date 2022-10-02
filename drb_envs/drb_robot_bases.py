import pybullet, pybullet_data
import gym, gym.spaces, gym.utils
import numpy as np
import os, inspect
# from KStand import KClass
# import rospy, actionlib
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class XmlBasedRobot:
  """
	Base class for mujoco .xml based agents.
	"""

  self_collision = True

  def __init__(self, robot_name, action_dim, obs_dim, self_collision):
    self.link_dict = None
    self.objects = []
    self.jdict = None
    self.ordered_joints = None
    self.robot_body = None

    lowActionSpace= np.array([
      -0.523599 #0 CHESTYP
      , -0.523599 #1
      , -0.524 #2
      , -0.523599  #3 HEADYP
      , -3.14159, #4 RSP
      -1.74533 , #5 RSR
      -3.14159 , #6 RSY
      -2.26893 , #7
      -2.00713 , #8
      -1.5708 , #9 RWP
      -3.14159,#10 RWY
      -1.0472 , #11 RH
      -3.14159 #12 LSP
      , 0.0 #13 LSR
      , -3.14159 
      , -2.26893 #14
      , -2.00713 #15
      , -1.5708 
      , -3.14159 
      , -1.0472 
      , -3.14159 
      , -0.523599 
      , -1.91986
      ,0.0
      , -0.523599, #LAR
      -1.74533, #LAP
      -3.14159, #RCY
      -0.523599, #RCR
      -1.91986, #RCP
      0.0, #RKP
      -0.523599 , #RAR
      -1.74533 #31 RAP
      ])
      
    highActionSpace = np.array([
        0.523599 #0
        , 1.22173 #1
        , 0.524 #2
        , 1.0472 #CHESTYP #HEAD_YP
        , 3.14159
      , 0.0
      , 3.14159
      , 0.0
      , 2.00713
      , 0.959931
      , 3.14159
      , 2.16421
      , 3.14159
      , 1.74533
      , 3.14159
      , 0.0
      , 2.00713
      , 0.959931
      , 3.14159
      , 2.16421
      , 3.14159
      , 0.523599
      , 0.523599
      , 2.44346
      , 0.523599
      , 0.698132
      , 3.14159
      , 0.523599
      , 0.523599
      , 2.44346
      , 0.523599 #30 RAR
      , 0.698132 #31  RAP
    ])    

    # for i in (range(1)):#, lowActionSpace): #zip range because joint type not iterable
    # high = np.deg2rad(366)*np.ones([action_dim])
    high = np.ones([action_dim])
   # high = highActionSpace[i]*np.ones([action_dim])
    # loow = lowActionSpace[i]*np.ones([action_dim])

    self.action_space = gym.spaces.Box(-high,high)
    high = np.inf * np.ones([obs_dim])
    self.observation_space = gym.spaces.Box(-high, high)
    print("robo base Action Space:", self.action_space)
    #self.model_xml = model_xml
    self.robot_name = robot_name
    self.self_collision = self_collision

  def addToScene(self, bullet_client, bodies):
    self._p = bullet_client

    # if self.parts is not None:
    #   parts = self.parts
    # else:
    link_dict = {}

    if self.jdict is not None:
      joints = self.jdict
    else:
      joints = {}

    if self.ordered_joints is not None:
      ordered_joints = self.ordered_joints
    else:
      ordered_joints = []

    if np.isscalar(bodies):  # streamline the case where bodies is actually just one body
      bodies = [bodies]

    dump = 0

    for i in range(len(bodies)): #len(bodies) is 1 || (bodies) is (0,)
      if self._p.getNumJoints(bodies[i]) == 0: #changing this brings KeyError 'floor' from self.ground_ids
        # print ("--if self._p.getNumJoints(bodies[i]) == 0--   ")
        link_name, robot_name = self._p.getBodyInfo(bodies[i])  #floor
        #-------name from URDF #link_name is floor || robot_name (sdf model_name) is b'floor_obj'
        self.robot_name = robot_name.decode("utf8")
        link_name = link_name.decode("utf8")
        link_dict[link_name] = BodyPart(self._p, link_name, bodies, 0, -1)#i, -1) #at this point BPIndex is still just 1 value, item>
        #----- dict of names from URDF |||  {'floor': <drb_robot_bases.B0dyP4rt object at 0x7f9964a15908>}
        #B0dyPrt's init(self, bullet_client, body_name, bodies, bodyIndex, bodyPartIndex)
        # print ('len(bodies): ', link_dict)
        # print ("r bases",bodies[1])

      for j in range(self._p.getNumJoints(bodies[i])):
        # self._p.setJointMotorControl2(bodies[i],
        #                               j,
        #                               pybullet.PD_CONTROL,
        #                               positionGain=0.1,
        #                               velocityGain=0.1,
        #                               force=0)

        jointInfo = self._p.getJointInfo(bodies[i], j) #bodyUniqueID, JOINTiNDEX
        joint_name = jointInfo[1] #joint name string from URDF
        link_name = jointInfo[12] #link name from URDF
        # joint_type= jointInfo[2] #[0] means revolute, [4] is FIXED
        joint_name = joint_name.decode("utf8")
        link_name = link_name.decode("utf8")
        if dump: print("LINK name:  '%s'" % link_name)
        if dump: print("JOINT name: '%s'" % joint_name)  # limits = %+0.2f..%+0.2f effort=%0.3f speed=%0.3f" % ((joint_name,) + j.limits()) )

        link_dict[link_name] = BodyPart(self._p, link_name, bodies, i, j) 
        #print here shows all links with robot_name as 2nd item

        if link_name == self.robot_name: #robot_name is set in KStand.py
          print("if link_name == self.robot_name")
          self.robot_body = link_dict[link_name] #===== getBodyInfo
          print("self.robot_body: ", self.robot_body)

        # if i == 0 and j == 0 and #works without this
        if self.robot_body is None:  # if nothing else works, we take this as robot_body
          # print ("robot_bases if i == 0 and j == 0 and self.robot_body is None ") 
          # #robotname is never link name
          link_dict[self.robot_name] = BodyPart(self._p, self.robot_name, bodies, i,j)# 0, -1) #======= from -1 to 31
          #print("self.robot_body: ", link_dict) #print link_dict here goes until robot_name (2nd item)
          self.robot_body = link_dict[self.robot_name]
          # print("self.robot_body: ", link_dict)

        # NO DIFFerence
        # if joint_name[:6] == "ignore":
        #   Joint(self._p, joint_name, bodies, i, j).disable_motor()
        #   continue

        # jdict[n] KeyError if disabled 
        # if joint_name[:8] != "jointfix":# this if was ENABLED in OG
        joints[joint_name] = Joint(self._p, joint_name, bodies, i, j)
        ordered_joints.append(joints[joint_name])
        # joints[joint_name].power_coef = 100.0

        # TODO: Maybe we need this
        # joints[joint_name].power_coef, joints[joint_name].max_velocity = joints[joint_name].limits()[2:4] #no attrib limits
        # self.ordered_joints.append(joints[joint_name])
        # self.jdict[joint_name] = joints[joint_name] #TypeError NoneType does not support

    return link_dict, joints, ordered_joints, self.robot_body

  def reset_pose(self, position, orientation):
    self.link_dict[self.robot_name].reset_pose(position, orientation)

class MJCFBasedRobot(XmlBasedRobot):
  """
	Base class for mujoco .xml based agents.
	"""

  def __init__(self, model_xml, robot_name, action_dim, obs_dim, self_collision=True):
    XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, self_collision)
    self.model_xml = model_xml
    self.doneLoading = 0

  def reset(self, bullet_client):

    self._p = bullet_client
    #print("Created bullet_client with id=", self._p._client)
    if (self.doneLoading == 0):
      self.ordered_joints = []
      self.doneLoading = 1
      if self.self_collision:
        self.objects = self._p.loadMJCF(os.path.join(pybullet_data.getDataPath(), "mjcf", self.model_xml),
                                        flags=pybullet.URDF_USE_SELF_COLLISION |
                                        pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS |
                                        pybullet.URDF_GOOGLEY_UNDEFINED_COLORS )
        self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(
            self._p, self.objects)
      else:
        self.objects = self._p.loadMJCF(
            os.path.join(pybullet_data.getDataPath(), "mjcf", self.model_xml, flags = pybullet.URDF_GOOGLEY_UNDEFINED_COLORS))
        self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(
            self._p, self.objects)
    self.robot_specific_reset(self._p)

    s = self.calc_state(
    )  # optimization: calc_state() can calculate something in self.* for calc_potential() to use

    return s

  def calc_potential(self):
    return 0

class URDFBasedRobot(XmlBasedRobot):
  """
	Base class for URDF .xml based robots.
	"""

  def __init__(self, model_urdf, robot_name, action_dim, obs_dim,
               basePosition=[0, 0, 0.955], #KHI_DEMO_HEIGHT
              #  basePosition=[0, 0, 1], #STRAIGHT
               baseOrientation=[0, 0, 0, 1],
               fixed_base=False,
               self_collision=1
              ):
    XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, self_collision)

    self.model_urdf = model_urdf
    self.basePosition = basePosition
    self.baseOrientation = baseOrientation
    self.fixed_base = fixed_base
    self.doneLoading = 0 #stolen from Class MJCFBasedRobot

  def reset(self, bullet_client):
    self._p = bullet_client
    # print("--- robot_bases ---") #disabled cuz it prints a lot
    # print("Created bullet_client with id=", self._p._client)
    # print(os.path.join(os.path.dirname(__file__), "data", self.model_urdf))
    if (self.doneLoading == 0):
      self.ordered_joints = []
      self.doneLoading= 1
      if self.self_collision:
        self.objects = self._p.loadURDF(os.path.join(pybullet_data.getDataPath(), self.model_urdf), basePosition=self.basePosition, baseOrientation=self.baseOrientation
                            ,useFixedBase=0
                            ,flags=pybullet.URDF_USE_SELF_COLLISION 
                            # | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
                            |pybullet.URDF_MERGE_FIXED_LINKS  #no need since 32 revoolute
                            )
       
      else:
        self.objects = self._p.loadURDF(os.path.join(pybullet_data.getDataPath(), self.model_urdf), basePosition=self.basePosition, baseOrientation=self.baseOrientation,
                            useFixedBase=0,
                            flags=pybullet.URDF_GOOGLEY_UNDEFINED_COLORS | pybullet.URDF_MERGE_FIXED_LINKS)
                            
      self.link_dict, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p, self.objects)
                            
    self.robot_specific_reset(self._p)
    s = self.calc_state()  # optimization: calc_state() can calculate something in self.* for calc_potential() to use
    self.potential = self.calc_potential()

    return s

  def calc_potential(self):
    return 0

class SDFBasedRobot(XmlBasedRobot):
  """
	Base class for SDF robots in a Scene.
	"""

  def __init__(self,
               model_sdf,
               robot_name,
               action_dim,
               obs_dim,
               basePosition=[0, 0, 0],
               baseOrientation=[0, 0, 0, 1],
               fixed_base=False,
               self_collision=False):
    XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, self_collision)

    self.model_sdf = model_sdf
    self.fixed_base = fixed_base

  def reset(self, bullet_client):
    self._p = bullet_client

    self.ordered_joints = []

    self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(
        self._p,  # TODO: Not sure if this works, try it with kuka
        self._p.loadSDF(os.path.join("models_robot", self.model_sdf)))

    self.robot_specific_reset(self._p)

    s = self.calc_state(
    )  # optimization: calc_state() can calculate something in self.* for calc_potential() to use
    self.potential = self.calc_potential()

    return s

  def calc_potential(self):
    return 0

class BodyPart:

  def __init__(self, bullet_client, body_name, bodies, bodyIndex, bodyPartIndex):
    self.bodies = bodies
    self._p = bullet_client
    self.bodyIndex = bodyIndex
    self.bodyPartIndex = bodyPartIndex
    # self.initialPosition = self.current_position()
    # self.initialOrientation = self.current_orientation()
    # self.PoseHelperClass = Pose_Helper(self)

  # def get_position(self):
  #   return self.current_position()
  # def xyz(self):
  #   return self.current_position()
  # def current_position(self):
  #   return self.get_pose()[:3]
  # def get_pose(self):
  #   return self.get_pos_orient(self.bodies[self.bodyIndex], self.bodyPartIndex)  #state_fields_of_pose_of()
  # def get_pos_orient(self, body_id, link_id): #=-1):  # a method you will most probably need a lot to get pose and orientation
  #   if link_id == -1:
  #     (x, y, z), (a, b, c, d) = self._p.getBasePositionAndOrientation(body_id)
  #   else:
  #     (x, y, z), (a, b, c, d), _, _, _, _ = self._p.getLinkState(body_id, link_id)
  #   return np.array([x, y, z, a, b, c, d]) 

  # def orientation(self):
  #   return self.current_orientation()
  # def get_orientation(self):
  #   return self.current_orientation()
  # def rpy(self):
  #   return pybullet.getEulerFromQuaternion(self.current_orientation())  
  # def current_orientation(self):
  #   return self.get_pose()[3:]

  def speed(self):
    if self.bodyPartIndex == -1:
      (vx, vy, vz), _ = self._p.getBaseVelocity(self.bodies[self.bodyIndex])
    else:
      (x, y, z), (a, b, c, d), _, _, _, _, (vx, vy, vz), (vr, vp, vy) = self._p.getLinkState(
          self.bodies[self.bodyIndex], self.bodyPartIndex, computeLinkVelocity=1)
    return np.array([vx, vy, vz])

  def reset_position(self, position):
    self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, self.get_orientation())

  def reset_orientation(self, orientation):
    self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], self.get_position(), orientation)

  def reset_velocity(self, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0]):
    self._p.resetBaseVelocity(self.bodies[self.bodyIndex], linearVelocity, angularVelocity)

  def reset_pose(self, position, orientation):
    self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)

  # def PHClassfunc(self):
  #   return self.PoseHelperClass

  # def contact_list(self):
  #   return self._p.getContactPoints(self.bodies[self.bodyIndex], -1, self.bodyPartIndex, -1)

class Joint:

  def __init__(self, bullet_client, joint_name, bodies, bodyIndex, jointIndex):
    self.bodies = bodies
    self._p = bullet_client
    self.bodyIndex = bodyIndex
    self.jointIndex = jointIndex
    self.joint_name = joint_name

    jointInfo = self._p.getJointInfo(self.bodies[self.bodyIndex], self.jointIndex)
    self.lowerLimit = jointInfo[8]
    self.upperLimit = jointInfo[9]
    self.jointMaxVel = jointInfo[11]
    self.jointMaxForce = jointInfo[10]

    self.power_coeff = 0

  def set_state(self, x, vx):
    self._p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, x, vx)

  def current_position(self):  # just some synonyme method
    return self.get_state()

  def current_relative_position(self):
    pos, vel, eff = self.get_state()
    pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
    if self.upperLimit == self.lowerLimit: #because ZeroDivisionError: float division by zero
      return (0, 0.1 * vel)
    else: return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), 0.1 * vel) #THIS SCALES POSITION TO B/W ) 0 and +-1

  def relative_PV(self):
    pos, vel, eff = self.get_state()
    vel_scaled = vel / self.jointMaxVel
    pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
    if self.upperLimit == self.lowerLimit: #because ZeroDivisionError: float division by zero
      return (0, vel_scaled)
    else: return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), vel_scaled)

  def relative_FVP(self):
    pos, vel, eff = self.get_state()
    vel_scaled = vel / self.jointMaxVel
    eff_scaled = eff/ (self.jointMaxForce * 0.3)
    pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
    if self.upperLimit == self.lowerLimit: #because ZeroDivisionError: float division by zero
      return (eff_scaled, vel_scaled, 0)
    else: return ( eff_scaled, vel_scaled, (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit) ) )


  def relative_Pos_Vel_Torq(self):
    pos, vel, eff = self.get_state()
    vel_scaled = vel / self.jointMaxVel
    eff_scaled = eff/ (self.jointMaxForce * 0.3)
    pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
    if self.upperLimit == self.lowerLimit: #because ZeroDivisionError: float division by zero
      return (0, vel_scaled, eff_scaled)
    else: return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), vel_scaled, eff_scaled)

  def ROS_relative_position(self):
    pos, vel = self.get_ROS_state()
    pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
    if self.upperLimit == self.lowerLimit: #because ZeroDivisionError: float division by zero
      return (0, 0.1 * vel)
    else: return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), 0.1 * vel)
  def get_ROS_state(self):
    x, vx = 1#self._p.getJointState(self.bodies[self.bodyIndex], self.jointIndex)
    return x, vx

  def get_state(self):
    x, vx, _, nM = self._p.getJointState(self.bodies[self.bodyIndex], self.jointIndex)
    return x, vx, nM

  def get_position(self):
    x, _ = self.get_state()
    return x
    
  def get_orientation(self):
    _, r = self.get_state()
    return r

  def get_velocity(self):
    _, vx = self.get_state()
    return vx

  def set_position(self, position, torque, vel):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex], self.jointIndex,
                                  pybullet.POSITION_CONTROL, targetPosition=position, force=torque, maxVelocity=vel ) 

  def set_positionNoVel(self, position, torque):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex], self.jointIndex,
                                  pybullet.POSITION_CONTROL, targetPosition=position, force=torque) 

  def positionPD(self, position, torque, vel, pgain, vgain):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex], self.jointIndex,
                                  pybullet.POSITION_CONTROL, targetPosition=position, force=torque, maxVelocity=vel, positionGain=pgain, velocityGain=vgain )                                   

  def pdControl(self, position, torque, pGain, vGain, vel):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex], self.jointIndex,
                                  pybullet.PD_CONTROL
                                  ,targetPosition=position
                                  ,force=torque #because TypeError setposition() takes 2 positional args but 3 were given_set_position,
                                  ,positionGain=pGain, velocityGain=vGain, maxVelocity=vel                                  
                                  )

  def set_velocity(self, velocity,torque):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex], self.jointIndex,
                                  pybullet.VELOCITY_CONTROL, targetVelocity=velocity, force=torque)

  def set_motor_torque(self, torque):  # just some synonyme method
    self.set_torque(torque)

  def set_torque(self, torque):
    self._p.setJointMotorControl2(bodyIndex=self.bodies[self.bodyIndex],
                                  jointIndex=self.jointIndex,
                                  controlMode=pybullet.TORQUE_CONTROL,
                                  force=torque)  

  def reset_current_position(self, position, velocity):
    self._p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, targetValue=position, targetVelocity=velocity)
    self.disable_motor() #WORKS WITHOUT THIS TOO!!! #BAD MOVE THO

  def disable_motor(self):
    self._p.setJointMotorControl2(self.bodies[self.bodyIndex],
                                  self.jointIndex,
                                  controlMode=pybullet.POSITION_CONTROL,
                                  targetPosition=0,
                                  targetVelocity=0,
                                  positionGain=0,
                                  velocityGain=0,
                                  force=0)

  def change_d(self, lateral_fric, linear_damp):
    self._p.changeDynamics(self.bodies[self.bodyIndex], self.jointIndex,
                            lateralFriction=lateral_fric, linearDamping=linear_damp)
