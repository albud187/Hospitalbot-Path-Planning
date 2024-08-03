#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from gymnasium.envs.registration import register
from hospital_robot_spawner.hospitalbot_env import HospitalBotEnv
from hospital_robot_spawner.hospitalbot_simplified_env import HospitalBotSimpleEnv
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
import os
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

class TrainingNode(Node):

    def __init__(self):
        super().__init__("hospitalbot_training", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Defines which action the script will perform "random_agent", "training", "retraining" or "hyperparam_tuning"
        self._training_mode = "training"

        # Get training parameters from Yaml file
        #self.test = super().get_parameter('test').value
        #self.get_logger().info("Test parameter: " + str(self.test))

def main(args=None):
    # Initialize the training node to get the desired parameters
    rclpy.init()
    node = TrainingNode()
    node.get_logger().info("Training node has been created")

    # Create the dir where the trained RL models will be saved
  
    pkg_dir = '/ros2_ws/training'
    trained_models_dir = os.path.join(pkg_dir, 'rl_models')
    log_dir = os.path.join(pkg_dir, 'logs')

    # If the directories do not exist we create them
    if not os.path.exists(trained_models_dir):
        os.makedirs(trained_models_dir)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Register the gym environment created in hospitalbot_env module

    env = HospitalBotSimpleEnv()
    # Here we check if the custom gym environment is fine
    check_env(env)
    node.get_logger().info("Environment check finished")

    # Now we create two callbacks which will be executed during training
    stop_callback = StopTrainingOnRewardThreshold(reward_threshold=900, verbose=1)
    eval_callback = EvalCallback(env, callback_on_new_best=stop_callback, eval_freq=100000, best_model_save_path=trained_models_dir, n_eval_episodes=40)

    ## Train the model
    model = PPO("MlpPolicy", 
                env, 
                verbose=1, 
                tensorboard_log=log_dir, 
                n_steps=20480, 
                gamma=0.9880614935504514, 
                gae_lambda=0.9435887928788405, 
                ent_coef=0.00009689939917928778, 
                vf_coef=0.6330533453055319, 
                learning_rate=0.00001177011863371444, 
                clip_range=0.1482)
    
    # Execute training
    try:
        model.learn(total_timesteps=int(100), 
                    reset_num_timesteps=False, 
                    callback=eval_callback, 
                    tb_log_name="PPO_test-1k",
                    progress_bar=True)
    except KeyboardInterrupt:
        model.save(f"{trained_models_dir}/PPO_test_1")
    # Save the trained model
    model.save(f"{trained_models_dir}/PPO_test")


    # Shutting down the node
    node.get_logger().info("The training is finished, now the node is destroyed")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()