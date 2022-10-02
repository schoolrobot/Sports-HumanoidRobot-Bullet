import sys, os
sys.path.append(os.path.dirname(__file__))
import pybullet, pybullet_data, gym

class Scene:
  "A base class for single- and multiplayer scenes"

  def __init__(self, bullet_client, gravity, timestep, frame_skip):
    self._p = bullet_client
    self.np_random, seed = gym.utils.seeding.np_random(None)
    self.timestep = timestep
    self.frame_skip = frame_skip

    self.dt = self.timestep * self.frame_skip
    self.cpp_world = World(self._p, gravity, timestep, frame_skip)

    self.test_window_still_open = True  # or never opened
    self.human_render_detected = False  # if user wants render("human"), we open test window

    self.multiplayer_robots = {}

  def test_window(self):
    "Call this function every frame, to see what's going on. Not necessary in learning."
    self.human_render_detected = True
    return self.test_window_still_open

  def actor_introduce(self, robot):
    "Usually after scene reset"
    if not self.multiplayer: return
    self.multiplayer_robots[robot.player_n] = robot

  def actor_is_active(self, robot):
    """
        Used by robots to see if they are free to exclusiveley put their HUD on the test window.
        Later can be used for click-focus robots.
        """
    return not self.multiplayer

  def episode_restart(self, bullet_client):
    "This function gets overridden by specific scene, to reset specific objects into their start positions"
    self.cpp_world.clean_everything()
    #self.cpp_world.test_window_history_reset()

  def global_step(self):
    """
        The idea is: apply motor torques for all robots, then call global_step(), then collect
        observations from robots using step() with the same action.
        """
    self.cpp_world.step(self.frame_skip)


class SingleRobotEmptyScene(Scene):
  multiplayer = False  # this class is used "as is" for InvertedPendulum, Reacher

# ---------------------------- copied from drb_scene_stadium.py --------------------
class StadiumScene(Scene):
  zero_at_running_strip_start_line = True  # if False, center of coordinates (0,0,0) will be at the middle of the stadium
  stadium_halflen = 105 * 0.25  # FOOBALL_FIELD_HALFLEN
  stadium_halfwidth = 50 * 0.25  # FOOBALL_FIELD_HALFWID
  stadiumLoaded = 0

  def episode_restart(self, bullet_client):
    self._p = bullet_client
    Scene.episode_restart(self, bullet_client)  # contains cpp_world.clean_everything()
    if (self.stadiumLoaded == 0):
      self.stadiumLoaded = 1

      # stadium_pose = cpp_household.Pose()
      # if self.zero_at_running_strip_start_line:
      #	 stadium_pose.set_xyz(27, 21, 0)  # see RUN_STARTLINE, RUN_RAD constants

      self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
      # self.ground_plane_mjcf= self._p.loadSDF("court_no_collision.sdf")
      self.ground_plane_mjcf= self._p.loadSDF("plane_stadium.sdf")

      # filename = os.path.join(pybullet_data.getDataPath(), "plane_stadium.sdf")
      # self.ground_plane_mjcf = self._p.loadSDF(filename)

      #filename = os.path.join(pybullet_data.getDataPath(),"stadium_no_collision.sdf")
      #self.ground_plane_mjcf = self._p.loadSDF(filename)
      #
      # for i in self.ground_plane_mjcf:
      #   self._p.changeDynamics(i, -1, lateralFriction= 1, restitution= 1)
        # self._p.changeVisualShape(i, -1, rgbaColor=[1,1,1, 0.8])
        # self._p.configureDebugVisualizer(pybullet.COV_ENABLE_PLANAR_REFLECTION,i)

        # for j in range(self._p.getNumJoints(i)):
        #   self._p.changeDynamics(i,j,lateralFriction=1)
      #despite the name (stadium_no_collision), it DID have collision, so don't add duplicate ground

class SinglePlayerStadiumScene(StadiumScene):
  "This scene created by environment, to work in a way as if there was no concept of scene visible to user."
  multiplayer = False

class MultiplayerStadiumScene(StadiumScene):
  multiplayer = True
  players_count = 3

  def actor_introduce(self, robot):
    StadiumScene.actor_introduce(self, robot)
    i = robot.player_n - 1  # 0 1 2 => -1 0 +1
    robot.move_robot(0, i, 0)


class World:

  def __init__(self, bullet_client, gravity, timestep, frame_skip):
    self._p = bullet_client
    self.gravity = gravity
    self.timestep = timestep
    self.frame_skip = frame_skip
    self.numSolverIterations = 30
    self.clean_everything()

  def clean_everything(self):
    #p.resetSimulation()
    self._p.setGravity(0, 0, -self.gravity)
    # self._p.connect(self._p.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.3')

    self._p.setDefaultContactERP(0.9)
    #print("self.numSolverIterations=",self.numSolverIterations)
    self._p.setPhysicsEngineParameter(fixedTimeStep=self.timestep * self.frame_skip,
                                      numSolverIterations=self.numSolverIterations,
                                      numSubSteps=self.frame_skip
                                      # ,useSplitImpulse = 100
                                      # ,splitImpulsePenetrationThreshold = 0.1
                                      # ,collisionFilterMode=1
                                      # ,jointFeedbackMode=self._p.JOINT_FEEDBACK_IN_WORLD_SPACE
                                      )

  def step(self, frame_skip):
    self._p.stepSimulation()
    # self._p.setRealTimeSimulation(1)

