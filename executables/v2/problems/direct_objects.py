from .base_problem import Problem
import random
import os
import numpy as np


class DirectObjects(Problem):
    # prediction directly from object properties
    def __init__(self, name, config):
        self.name = name
        self.data_path = config['data_path']
        self.test_data_path = config['test_data_path']
        # self.train_success_folder = config['train_success_folder']
        # self.train_fail_folder = config['train_fail_folder']
        # self.eval_success_folder = config['eval_success_folder']
        # self.eval_fail_folder = config['eval_fail_folder']

        self.scene_goals = config['scene_goals']
        self.balance_data = config['balance_data']
        
        train_success_folders = []
        train_fail_folders = []

        # problem specific
        env_config_folders = os.listdir(self.data_path)
        test_env_configs = os.listdir(self.test_data_path)
        
        random.shuffle(env_config_folders)
        train_env_configs = env_config_folders[:300]
        eval_env_configs = env_config_folders[350:]
        
        for env_config_folder in train_env_configs:
            train_path = os.path.join(self.data_path, env_config_folder)
            train_success_folders.extend([os.path.join(train_path, "success", folder) for folder in os.listdir(os.path.join(train_path, "success"))])
            train_fail_folders.extend([os.path.join(train_path, "failure", folder) for folder in os.listdir(os.path.join(train_path, "failure"))])

        if self.balance_data:
            min_len_data = min(len(train_success_folders), len(train_fail_folders))
            train_success_folders = random.sample(train_success_folders, min_len_data)
            train_fail_folders = random.sample(train_fail_folders, min_len_data)
       
        self.balance_ratio = len(train_success_folders) / len(train_fail_folders)
        self.training_data = train_success_folders + train_fail_folders
        
        eval_success_folders = []
        eval_fail_folders = []
        for env_config_folder in eval_env_configs:
            eval_path = os.path.join(self.data_path, env_config_folder)
            eval_success_folders.extend([os.path.join(eval_path, "success", folder) for folder in os.listdir(os.path.join(eval_path, "success"))])
            eval_fail_folders.extend([os.path.join(eval_path, "failure", folder) for folder in os.listdir(os.path.join(eval_path, "failure"))])

        if self.balance_data:
            min_len_data = min(len(eval_success_folders), len(eval_fail_folders))
            eval_success_folders = random.sample(eval_success_folders, min_len_data)
            eval_fail_folders = random.sample(eval_fail_folders, min_len_data)
       
        self.evaluation_data = eval_success_folders + eval_fail_folders

        test_data = []
        for env_config_folder in test_env_configs:
            test_path = os.path.join(self.test_data_path, env_config_folder)
            test_data.extend([os.path.join(test_path, "success", folder) for folder in os.listdir(os.path.join(test_path, "success"))])
            test_data.extend([os.path.join(test_path, "failure", folder) for folder in os.listdir(os.path.join(test_path, "failure"))])
        self.test_data = test_data

    def get_training_data(self):
        return self.training_data

    def get_evaluation_data(self):
        return self.evaluation_data
    
    def get_test_data(self):
        return self.test_data
    
    def get_balance_ratio(self):
        return self.balance_ratio
    
    def get_name(self):
        return self.name


    