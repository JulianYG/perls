from gym.envs.registration import registry, register, make, spec

# ------------bullet-------------

register(
    id='GraspBulletEnv-v0',
    entry_point='bullet.env:GraspBulletEnv',
    timestep_limit=1000,
    reward_threshold=950.0,
)
