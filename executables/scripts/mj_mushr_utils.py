import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# @torch.jit.script
class MjMushr():
    def __init__(self):
        self.DimX = 3     # [xd,yd,thd]
        self.DimU = 2     # [u0,u1]
        self.DimF = 3    # [xdd,ydd,thdd]

    def handle_horizon(self, horizon, states, controls, dt):

        xi = torch.empty(len(states)-horizon, self.DimX, horizon);
        us = torch.empty(len(controls)-horizon, self.DimU, horizon);
        fhat = torch.empty(len(states)-horizon, self.DimF, horizon);
        xH = torch.Tensor();
         # = torch.Tensor();
        # gr = torch.Tensor();

        # print(f"ctrls: {controls[:10]}")
        for idx in range(horizon):
            up  = torch.stack(controls[idx:-horizon+idx])
            # up1  = torch.stack(controls[idx+1:-horizon+idx+1])

            xip = torch.stack(  states[idx:-horizon+idx])
            # xip1 = torch.stack(  states[idx+1:-horizon+idx+1])
            # xip2 = torch.stack(  states[idx+2:-horizon+idx+2])

            # print(f"us: {us[:10]}")
            # xi = torch.stack((xi, xip),2)
            us[:,:,idx] = up
            xi[:,:,idx] = xip
        # print(f"xi: {xi[:10].shape} {xi[:10]}")
        # exit(-1)
        xH = torch.stack(states[horizon:])
        
        # xH = torch.hstack((xH0, xH1, xH2))

        # gi = torch.zeros_like(gr[:,0:1]);
        x0 = xi[:,:,0]
        # print(f"xi {xi.shape} ")
        # print(f"x0 {x0.shape} {x0[:10]} ")
        for h in range(horizon-1):
            xT = xi[:,:,h+1]
            xT0 = (xT - x0) / dt
            fhat[:,:,h] = xT0
        
        xT0 = (xH - x0) / dt
        # gi = gi + gr[:,(horizon-1):horizon];
        fhat[:,:,-1] = xT0;
        # print(f"fhat {fhat[:10]} ")
        # exit(-1)
  
        return xi, us, fhat

    @torch.jit.unused
    def from_file(self, filename, horizon=1, dt=0.1):
        f = open(filename, 'r')
        data_states = []
        data_controls = []
        data_target = []

        states = torch.empty(0, self.DimX, horizon);
        controls = torch.empty(0, self.DimU, horizon);
        targets = torch.empty(0, self.DimF, horizon);

        # print(f"states {states}")
        for line in f:
            ls = line.split();
            if len(ls) == 0:
                if len(data_states) > 0:
                    
                    xs, us, fs = self.handle_horizon(horizon, data_states, data_controls, dt)

                    # data_states_tensor = torch.stack(data_states)
                    # data_controls_tensor = torch.stack(data_controls)
                    # data_target_tensor = torch.stack(data_target)
                    # print(f"xs {xs.shape}")
                    states = torch.vstack((states, xs))
                    controls = torch.vstack((controls, us))
                    targets = torch.vstack((targets, fs))

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
