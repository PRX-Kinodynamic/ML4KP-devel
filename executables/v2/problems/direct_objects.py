from .base_problem import Problem
import random
import os


class DirectObjects(Problem):
    # prediction directly from object properties
    def __init__(self, name, config):
        self.name = name
        self.data_path = config['data_path']
        self.test_data_path = config['test_data_path']
        self.train_success_folder = config['train_success_folder']
        self.train_fail_folder = config['train_fail_folder']
        self.eval_success_folder = config['eval_success_folder']
        self.eval_fail_folder = config['eval_fail_folder']

        self.scene_goals = config['scene_goals']

        self.balance_data = config['balance_data']

        with open(self.train_success_folder, 'r') as f:
            train_success_folders = [os.path.join(self.data_path, line.strip()) for line in f.readlines()]
        with open(self.train_fail_folder, 'r') as f:
            train_fail_folders = [os.path.join(self.data_path, line.strip()) for line in f.readlines()]

        if self.balance_data:
            min_len_data = min(len(train_success_folders), len(train_fail_folders))
            train_success_folders = random.sample(train_success_folders, min_len_data)
            train_fail_folders = random.sample(train_fail_folders, min_len_data)
       
        self.balance_ratio = len(train_success_folders) / len(train_fail_folders)
        self.training_data = train_success_folders + train_fail_folders
        
        with open(self.eval_success_folder, 'r') as f:
            eval_success_folders = [os.path.join(self.data_path, line.strip()) for line in f.readlines()]
        with open(self.eval_fail_folder, 'r') as f:
            eval_fail_folders = [os.path.join(self.data_path, line.strip()) for line in f.readlines()]

        if self.balance_data:
            min_len_data = min(len(eval_success_folders), len(eval_fail_folders))
            eval_success_folders = random.sample(eval_success_folders, min_len_data)
            eval_fail_folders = random.sample(eval_fail_folders, min_len_data)
       
        self.evaluation_data = eval_success_folders + eval_fail_folders

        test_data = sorted(os.listdir(self.test_data_path), key=lambda x: int(x))
        self.test_data = []
        for k in range(len(test_data)//self.scene_goals):
            folder_data = []
            for i in range(self.scene_goals):
                folder_data.append(os.path.join(self.test_data_path, test_data[k*self.scene_goals + i]))
            self.test_data.append(folder_data)

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


    