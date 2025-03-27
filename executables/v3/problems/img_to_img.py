from .base_problem import Problem
import os
import yaml
from pathlib import Path
import random
import numpy as np

class ImgToImg(Problem):
    def __init__(self, name, config):
        self.name = name
        # with open(config, 'r') as f:
        #     config = yaml.safe_load(f)
        self.data_path = config['data_path']
        self.problem_type = config['problem_type']
        self.train_test_split = config['train_test_split']
        self._setup_data()
    
    def _setup_data(self):
        self.training_data = []
        self.evaluation_data = []
        
        all_data_files = [os.path.join(self.data_path, f) for f in os.listdir(self.data_path)]

        random.shuffle(all_data_files)
        train_data_files = all_data_files[:int(len(all_data_files) * self.train_test_split)]
        eval_data_files = all_data_files[int(len(all_data_files) * self.train_test_split):]
        
        self.training_data = train_data_files
        self.evaluation_data = eval_data_files
        
    def get_name(self):
        return self.name
    
    def get_training_data(self):
        return self.training_data
    
    def get_evaluation_data(self):
        return self.evaluation_data
    
    def get_test_data(self):
        """Return test data for evaluation"""
        # This method is missing but is called in the interface code
        return self.evaluation_data  # This would be a basic implementation
    
    def get_balance_ratio(self):
        """Return the balance ratio for positive/negative samples"""
        # This method is missing
        return None  # Default implementation could return None
    
if __name__ == "__main__":
    problem = ImgToImg(name="single_env_goal", config="executables/v3/problems/configs/single_env_goal.yaml")
   