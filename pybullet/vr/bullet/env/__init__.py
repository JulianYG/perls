from gym.envs.registration import registry, register, make, spec

# ------------bullet-------------

register(
    id='RobotBulletEnv-v0',
    entry_point='bullet.env:RobotBulletEnv',
    timestep_limit=1000,
    reward_threshold=950.0,
)
