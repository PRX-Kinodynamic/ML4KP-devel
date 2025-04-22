import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# @torch.jit.script
class MjMushr():
    def __init__(self):
        self.DimX = 3     # [xd,yd,thd]
        self.DimU = 2     # [u0,u1]
        self.DimF = 3    # [xdd,ydd,thdd]

        # self.xd_max = 4.0
        # self.yd_max = 2.0
        # self.thd_max = 4.0
        # self.u0_max = 0.25
        # self.u1_max = 0.5

        # self.xdd_max = 4.0
        # self.ydd_max = 4.0
        # self.thdd_max = 9.0


    @torch.jit.unused
    def from_file(self, filename):
        f = open(filename, 'r')
        data_states = []
        data_controls = []
        data_target = []

        states = torch.Tensor();
        controls = torch.Tensor();
        targets = torch.Tensor();

        for line in f:
            ls = line.split();
            if len(ls) == 0:
                if len(data_states) > 0:
                    data_states_tensor = torch.stack(data_states)
                    data_controls_tensor = torch.stack(data_controls)
                    data_target_tensor = torch.stack(data_target)
                    states = torch.cat((states, data_states_tensor), 0)
                    controls = torch.cat((controls, data_controls_tensor), 0)
                    targets = torch.cat((targets, data_target_tensor), 0)
                    data_states = []
                    data_controls = []
                    data_target = []
            else:   
                xd = float(ls[4])  
                yd = float(ls[5])  
                thd = float(ls[6]) 

                xdd = float(ls[7])  
                ydd = float(ls[8])  
                thdd = float(ls[9]) 

                u0 = float(ls[10])  
                u1 = float(ls[11])  

                state = torch.Tensor([xd, yd, thd])
                control = torch.Tensor([u0, u1])
                target = torch.Tensor([xdd, ydd, thdd])
                
                data_states.append(state)
                data_controls.append(control)
                data_target.append(target)
                
        return states, controls, targets
