from gym.envs.registration import register

register(
    id='perls-v0',
    entry_point='gym_perls.envs:PerlsEnv',
    kwargs=dict(conf='gym.xml'),
)
# register(
#     id='foo-extrahard-v0',
#     entry_point='gym_foo.envs:FooExtraHardEnv',
# )

