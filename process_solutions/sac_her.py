from env import *
import sys
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3 import HerReplayBuffer, SAC, TD3, DDPG
from stable_baselines3.common.noise import NormalActionNoise
plt.rcParams['figure.figsize'] = [8, 8]
alg = SAC

# env_name = sys.argv[1]
for width in [1]:
    yaml_file=f"/home/kushal/dirtmp/resources/input_files/environments/passage_{width}.yaml"
    models_path = f"./SAC-HER-Models/Passage-{width}-high-precision/"
    train_model = True
    num_epochs = int(1e6)

    env = PropPenalty(yaml_file=yaml_file, max_steps=100)
    policy_kwargs = {}
    model = SAC(
        "MultiInputPolicy",
        env,
        replay_buffer_class=HerReplayBuffer,
        replay_buffer_kwargs=dict(
            goal_selection_strategy='future',
            online_sampling=False,
            max_episode_length=100,
            n_sampled_goal=4,
        ),
        learning_rate=3e-4,
        policy_kwargs=dict(net_arch=[256, 256]),
        learning_starts=1000,
        buffer_size=int(1e6),
        verbose=1,
        gamma=0.95,
    )

    if train_model:
        eval_callback = EvalCallback(env,n_eval_episodes=100,eval_freq=num_epochs/100,best_model_save_path=models_path,
                                log_path=models_path+"logs",deterministic=False)
        model.learn(num_epochs,callback=eval_callback)
    else:
        model.model = SAC.load(models_path+"/best_model",env=env)