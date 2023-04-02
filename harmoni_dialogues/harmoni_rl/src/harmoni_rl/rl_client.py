import os
import ast
import pandas as pandas
import numpy as np
import d3rlpy
import gym
import rospy

class RLActionsName:
    ACTION1 = "Followup"
    ACTION2 = "NewEpisode"
    ACTION3 = "RepairStrategy"
 


class RLCore:
    def setup():
        env = gym.make("custom")
        eval_env = gym.make("custom")
        dqn = d3rlpy.algos.DQN(
            batch_size = 32,
            learning_rate = 2.5e-4,
            target_update_interval = 100,
            use_gpu = False,
        )
        dqn.build_with_env(env)
        # experience replay buffer
        buffer = d3rlpy.online.buffers.ReplayBuffer(maxlen=100000, env=env)

        # exploration strategy
        # in this tutorial, epsilon-greedy policy with static epsilon=0.3
        explorer = d3rlpy.online.explorers.ConstantEpsilonGreedy(0.3)

    def start_training():
        dqn.fit_online(
            env,
            buffer,
            explorer,
            n_steps=100000,  # train for 100K steps
            eval_env=eval_env,
            n_steps_per_epoch=1000,  # evaluation is performed every 1K steps
            update_start_step=1000,  # parameter update starts after 1K steps
        )

    def test():
        rospy.sleep(4)
        return RLActionsName.ACTION1
