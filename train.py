#!/usr/bin/env python

from mpi4py import MPI
from baselines.common import set_global_seeds, tf_util as U
from baselines import bench
import os.path as osp
import gym, logging
from baselines import logger
import perls

def train(env_id, num_timesteps, seed):

    from baselines.ppo1 import pposgd_simple, mlp_policy
    import baselines.common.tf_util as U

    rank = MPI.COMM_WORLD.Get_rank()
    U.single_threaded_session().__enter__()

    if rank != 0: logger.set_level(logger.DISABLED)
    workerseed = seed + 10000 * MPI.COMM_WORLD.Get_rank()
    
    set_global_seeds(workerseed)

    env = gym.make(env_id)

    policy = mlp_policy.MlpPolicy(name=name, 
            ob_space=ob_space, ac_space=ac_space, 
            hid_size=32, num_hid_layers=3)

    def policy_fn(name, ob_space, ac_space): #pylint: disable=W0613
        return policy

    env = bench.Monitor(env, logger.get_dir() and 
        osp.join(logger.get_dir(), "%i.monitor.json" % rank))

    env.seed(workerseed)
    gym.logger.setLevel(logging.WARN)

    pposgd_simple.learn(env, policy_fn,
        max_timesteps=num_timesteps,
        timesteps_per_batch=1024,
        clip_param=0.2, entcoeff=0.01,
        optim_epochs=4, optim_stepsize=1e-4, optim_batchsize=30,
        gamma=0.99, lam=0.95,
        schedule='linear'
    )
    env.close()

def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID', default='push-vel-v0')
    parser.add_argument('--seed', help='RNG seed', type=int, default=42)
    args = parser.parse_args()
    train(args.env, num_timesteps=1e5, seed=args.seed)

if __name__ == '__main__':
    main()


