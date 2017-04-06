from gym.envs.registration import registry, register, make, spec

# ------------bullet-------------

register(
    id='KukaBulletEnv-v0',
    entry_point='bullet.env:KukaBulletEnv',
    timestep_limit=1000,
    reward_threshold=950.0,
)
