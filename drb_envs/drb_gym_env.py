from os import stat
from .drb_scene_abstract import SinglePlayerStadiumScene
from .drb_env_bases import MJCFBaseBulletEnv
import numpy as np
import pybullet, time
from KStand import KClass, KFlagrunHarderClass, KFlagrunClass
from KSquat import KSquatClass
from KStand_18 import KClass18, KFlagrun18, KFlagrunHarder18
from KStand_22 import KClass22, KFlagrun22
from KStand_32 import KClass32

class WalkerBaseBulletEnv(MJCFBaseBulletEnv):

  def __init__(self, robot, render=False):
    print("--drb_env.py-- WalkerBase::__init__ start")
    self.camera_x = 0
    self.walk_target_x = 100  # kilometer away
    self.walk_target_y = 0
    self.stateId = -1
    MJCFBaseBulletEnv.__init__(self, robot, render)

  def create_single_player_scene(self, bullet_client):
    self.stadium_scene = SinglePlayerStadiumScene(bullet_client,
                                                  gravity=9.8,
                                                  # timestep=0.0165 / 4,
                                                  timestep=1/100,
                                                  frame_skip= 1 )
    return self.stadium_scene

  def reset(self):
    if (self.stateId >= 0):
      # print("restoreState self.stateId:",self.stateId)
      self._p.restoreState(self.stateId)

    r = MJCFBaseBulletEnv.reset(self)
    # self._p.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    self._p.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    self.parts, self.jdict, self.ordered_joints, self.robot_body = self.robot.addToScene(self._p, self.stadium_scene.ground_plane_mjcf)
    # MJCFBaseBulletEnv.render(self) #CAMERA func
    self._p.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
    if (self.stateId < 0):
      self.robot.noCollision()
      self.robot.myResetFunc()
      # self.robot.basketball()
      # self.robot.rosBridgeInitNode()

      # self.first_head= self.robot.getTorsoInitialHeights()
      # nbaball = self._p.loadURDF("/home/admin/3x3/nba.urdf", [0.2, -0.6, 1.50], globalScaling=1, useFixedBase=0)
      # self._p.addUserDebugText("NBA BALL", [0, 0,0.12], textSize=1.2, parentObjectUniqueId=nbaball, parentLinkIndex=-1, textColorRGB=[1.3,1.7,0])
      # self._p.changeDynamics(nbaball, -1, restitution=0 )
      self.stateId = self._p.saveState()
      print("saving state self.stateId:",self.stateId)

    return r

  def _isDone(self):
    return self._alive < 0

  def move_robot(self, init_x, init_y, init_z):
    "Used by multiplayer stadium to move sideways, to another running lane."
    self.cpp_robot.query_position()
    pose = self.cpp_robot.root_part.pose()
    pose.move_xyz( init_x, init_y, init_z )  # Works because robot loads around (0,0,0), and some robots have z != 0 that is left intact
    self.cpp_robot.set_pose(pose)

  electricity_cost = -10.0  # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
  stall_torque_cost = -0.1  # cost for running electric current through a motor even at zero rotational speed, small
  # foot_collision_cost = -1.0  # touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
  joints_at_limit_cost = -0.1  # discourage stuck joints

  def step(self, policyOut):
    if not self.scene.multiplayer:  # if multiplayer, action first applied to all robots, then global step() called, then _step() for all robots with the same actions
      self.robot.apply_action(policyOut)
      whereLook0 = self._p.getBasePositionAndOrientation(1); whereLook = whereLook0[0]
      self._p.resetDebugVisualizerCamera(2.3, 60, 0, [whereLook[0], whereLook[1], whereLook[2] ] )
      # obs_list = self.robot.calc_state()
      # obs_2 = set(obs_list)
      # obs_2.remove(max(obs_2))
      # print ("cal_state LENGTH: ", len(obs_2))
      # print ("MAX OBSERVATIONz: ", (max(obs_list)))
      # self.robot.choreonoidOut()
      self.scene.global_step()
    state = self.robot.calc_state()  # also calculates self.joints_at_limit
    # print ("new state:", self.robot.feet_on_ground)
    # self._p.addUserDebugText(self.robot.linkInfo()[0][2], [-0.0, -0.31, 0], textSize=2, parentObjectUniqueId=1, parentLinkIndex=31, textColorRGB=[1,0.3,0])

    # print(self.robot.linkInfo()[0][2])
    self._alive=(
                  #self.robot.NFLrules()
                  #min(self.robot.NFLrules(), self.robot.bodyPartHeights())
                  # self.robot.feetUnderAlive()
                  self.robot.waistEulerAlive()
                  #min(self.robot.imuAlive(), self.robot.waistAlive())
    )
    done = self._isDone()
    if not np.isfinite(state).all():
      print("~INF~", state)
      done = True

    potential_old = self.potential
    self.potential = self.robot.calc_potential()
    progress = float(self.potential - potential_old)

    electricity_cost = self.electricity_cost * float(np.abs(policyOut * self.robot.joint_speeds).mean()) # let's assume we have DC motor with controller, and reverse current braking    # ValueError: operands could not be broadcast together with shapes (26,) (34,) 
    electricity_cost += self.stall_torque_cost * float(np.square(policyOut).mean())
    joints_at_limit_cost = float(self.joints_at_limit_cost * self.robot.joints_at_limit)

    self.rewards = [
      self._alive
      #progress,
      ,electricity_cost
      ,joints_at_limit_cost
    ]    
    # print ("reward:", (self.rewards), "sum:", sum(self.rewards))
    # print (self.robot.flatFooted()[0] ,self.robot.flatFooted()[1])
    debugmode = 0
    if (debugmode):
      # print("--- in drb_gym_env.py ---") #cuz idk where what was
      # print("it's good to be alive: ",  self._alive)
      # print("i am making progress: ", progress)
      # print("electricity_cost: ", electricity_cost) #probably not in use cuz this + should ERROR
      # print("joints_at_limit_cost: ", joints_at_limit_cost) #yes it errored, + is for string
      # print("feet_collision_cost: ", feet_collision_cost)
      # print("my foot or feet ON GROUND: ", feet_ground_cost)
      # print("(not) touching ITSELF: ", touchSelf_cost)
      # print("i want to liiive!!", longLifeReward)
      # print("IMU reward", imuReward)
      # print("i am UPRIGHT (Straight)", stayingUpright)
      # print ("my FEET are FLAT!", feetAreFlat)
      # print("rewards list= ", self.rewards)
      # print("sum rewards: ", sum(self.rewards))
      self.HUD(state, policyOut, done)
    # self.reward += sum(self.rewards) #not used, why?

    # f = open('/home/admin/dribble_repo/rewards_log.txt','a') # Prints Action Values
    # for s in self.rewards:
    #   f.write(str(s))
    #   f.write(" ")
    # f.write("\n")
    # f.close()    
    # time.sleep(1/400)
    print (policyOut)
    return state, sum(self.rewards), bool(done), {}

  def camera_adjust(self):
    # x, y, z = self.robot.waistZRP
    # self.camera_x = x
    self.camera.move_and_look_at(0.5, 0, 45, self.robot.cameraWaist(), self.robot.cameraWaist(), self.robot.cameraWaist())

class KClassBulletEnv(WalkerBaseBulletEnv):
  def __init__(self, robot= None, render= 1):
    if robot is None: self.robot = KClass()
    else: self.robot = robot
    WalkerBaseBulletEnv.__init__(self, self.robot, render)
    self.electricity_cost = 1 * WalkerBaseBulletEnv.electricity_cost
    self.stall_torque_cost =1 * WalkerBaseBulletEnv.stall_torque_cost

class K18Env(WalkerBaseBulletEnv):
  def __init__(self, robot= None, render= 1):
    if robot is None: self.robot = KClass18()
    else: self.robot = robot
    WalkerBaseBulletEnv.__init__(self, self.robot, render)
    self.electricity_cost = 1 * WalkerBaseBulletEnv.electricity_cost
    self.stall_torque_cost =1 * WalkerBaseBulletEnv.stall_torque_cost

class K22Env(WalkerBaseBulletEnv):
  def __init__(self, robot= None, render= 1):
    if robot is None: self.robot = KClass22()
    # if robot is None: self.robot = KFlagrun22()
    # if robot is None: self.robot = KClass22()
    else: self.robot = robot
    WalkerBaseBulletEnv.__init__(self, self.robot, render)
    self.electricity_cost = 1 * WalkerBaseBulletEnv.electricity_cost
    self.stall_torque_cost =1 * WalkerBaseBulletEnv.stall_torque_cost


class KSquatEnv(WalkerBaseBulletEnv):
  def __init__(self, robot= None, render= 1):
    if robot is None: self.robot = KSquatClass()
    else: self.robot = robot
    WalkerBaseBulletEnv.__init__(self, self.robot, render)
    self.electricity_cost = 10 * WalkerBaseBulletEnv.electricity_cost
    self.stall_torque_cost = 10 * WalkerBaseBulletEnv.stall_torque_cost

class KFlagrunBulletEnv(KClassBulletEnv):
  random_yaw = True

  def __init__(self, render=1):
    self.robot = KFlagrunClass()
    KClassBulletEnv.__init__(self, self.robot, render)

  def create_single_player_scene(self, bullet_client):
    s = KClassBulletEnv.create_single_player_scene(self, bullet_client)
    s.zero_at_running_strip_start_line = False
    return s

class KFlagrunHarderBulletEnv(KClassBulletEnv):
  random_lean = True  # can fall on start

  def __init__(self, render=1):
    self.robot = KFlagrunHarderClass()
    self.electricity_cost /= 4  # don't care that much about electricity, just stand up!
    KClassBulletEnv.__init__(self, self.robot, render)

  def create_single_player_scene(self, bullet_client):
    s = KClassBulletEnv.create_single_player_scene(self, bullet_client)
    s.zero_at_running_strip_start_line = False
    return s