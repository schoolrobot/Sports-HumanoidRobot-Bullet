from gym.envs.registration import register

register(id= 'KClassBulletEnv-v0', entry_point= 'drb_envs.drb_gym_env:KClassBulletEnv'
         #max_episode_steps=1000,
         #reward_threshold=2500.0
)

register(id= 'K18Env-v0', entry_point= 'drb_envs.drb_gym_env:K18Env')
register(id= 'K22Env-v0', entry_point= 'drb_envs.drb_gym_env:K22Env')
register(id= 'KSquatEnv-v0', entry_point= 'drb_envs.drb_gym_env:KSquatEnv')

register(id='KFlagrunEnv-v0',
         entry_point='drb_envs.drb_gym_env:KFlagrunBulletEnv')

register(id='KFlagrunHarderEnv-v0',
         entry_point='drb_envs.drb_gym_env:KFlagrunHarderBulletEnv')
