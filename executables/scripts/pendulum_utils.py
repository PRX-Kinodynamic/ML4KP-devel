import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

@torch.jit.script
class Pendulum():
    def __init__(self):
        self.DimX = 1      # thd
        self.DimU = 1      # u
        self.DimF = 1      # accel

        # self.norm_input_tensor = torch.tensor([self.thd_max, self.u_max])
        # self.norm_output_tensor = torch.tensor([self.ft_max])
        # thd_max = 2 * 3.1415926536
        # u_max = 0.6371781908344007;

    # @torch.jit.export 
    # def normalize_input(self, thd_u):

    #     # thd_u[0] = thd_u[0] / thd_max
    #     # thd_u[1] = thd_u[1] / u_max
    #     # return thd_u

    #     # return torch.div(thd_u, self.norm_input_tensor)
    #     return thd_u

    # @torch.jit.export
    # def normalize_output(self, ft):
    #     # self.ft_max = 35
    #     # ft = ft / ft_max
    #     # if ft > 1.0:
    #     #     raise ValueError("Output out of bounds: " + str(ft))
    #     return torch.div(ft, self.norm_output_tensor)

    # @torch.jit.export
    # def unnormalize_output(self, ft):
    #     # ft_max = 35
    #     # ft = ft * ft_max
    #     return torch.mul(ft, self.norm_output_tensor);

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
                thd = float(ls[1])
                u = float(ls[2])
                ft = float(ls[5])

                thd = torch.Tensor([thd])
                u = torch.Tensor([u])
                ft = torch.Tensor([ft])

                data_states.append(thd)
                data_controls.append(u)
                data_target.append(ft)
        return states, controls, targets