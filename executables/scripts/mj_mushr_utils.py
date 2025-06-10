import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# @torch.jit.script
class MjMushr():
    def __init__(self):
        self.DimX = 3     # [xd,yd,thd]
        self.DimU = 2     # [u0,u1]
        self.DimF = 3    # [xdd,ydd,thdd]

    def handle_horizon(self, horizon, states, controls, dt, targets):

        xi = torch.empty(len(states)-horizon-1, horizon+1, self.DimX);
        us = torch.empty(len(controls)-horizon-1, horizon+1, self.DimU);
        fhat = torch.empty(len(states)-horizon-1, horizon, self.DimF);

        # print(f"ctrls: {controls[:10]}")
        for idx in range(horizon + 1):
            up  = torch.stack(controls[idx:-horizon-1+idx])
            xip = torch.stack(states[idx:-horizon-1+idx])
            # xip1 = torch.stack(  states[idx+1:-horizon+idx+1])
            # xip2 = torch.stack(  states[idx+2:-horizon+idx+2])

            # print(f"us: {us[:10]}")
            # xi = torch.stack((xi, xip),2)
            us[:,idx,:] = up
            xi[:,idx,:] = xip

        x0 = xi[:,0,:]
        # print(f"xi {xi.shape} ")
        # print(f"x0 {x0.shape} ")
        # print(f"x0 {x0} ")
        # print(f"fhat {fhat.shape} ")
        for idx in range(1,horizon+1):  
            fip = (xi[:,idx,:] - x0 ) / dt
            # fip_test = torch.stack(targets[idx:-horizon+idx])
            fhat[:,idx-1,:] = fip
                
            # print(f"idx {idx} fip {fip} ")
            # print(f"fip_test {fip_test} ")

        # print(f"fhat {fhat} ")
        # print(f"states {states[0:-horizon]} ")

        # xi[:,-1,:] = torch.stack(states[horizon:-horizon])

        # print(f"xi {xi.shape} ")
        # print(f"us {us.shape} ")
        # print(f"fhat {fhat.shape} ")
        # print(f"xi {xi[5,:]} ")
        # print(f"fhat {fhat[5,:]} ")
        # for h in range(horizon-1):
        #     # ha = -h-1
        #     # print(f"h {h} fhat {fhat[:,:,h]} ")
        #     # print(f"h {h} fhat-h {fhat[:,:,h]} ")
        #     # print(f"h-1 {h} fhat-h-1 {fhat[:,:,h+1]} ")
        #     fhat[:,h+1,:] += fhat[:,h,:]
        #     # fhat[:,h,:] += fhat[:,h+1,:]
        #     # print(f"h {h} fhat {fhat} ")
        # # print(f"xi {xi} ")
        # fhat = fhat * dt
        # print(f"fhat {fhat[5,:]} ")
        # xH = torch.stack(states[horizon:])
        
        # x0 = xi[:,:,0]
        # for h in range(horizon-1):
        #     xT = xi[:,:,h+1]
        #     xT0 = (xT - x0) / dt
        #     fhat[:,:,h] = xT0
        
        # xT0 = (xH - x0) / dt
        # fhat[:,:,-1] = xT0;
        # print(f"xi {xi.shape} fhat {fhat.shape}")
        # print(f"xi {xi[:2]} ")
        # print(f"xT0 {xT0[:,0]} ")
        # xi_aux = xi + fhat * dt
        # xz = xi[1:] - xi_aux[:-1]
        # print(f"xi_aux {xi_aux} ")
        # print(f"xz {xz} ")
        # zt = torch.allclose(xz, torch.zeros_like(xz), rtol=1e-5, atol=1e-5)
        # print(f"zt {zt}")
        # print(f"xi {xi} fhat {fhat}")

        # exit(-1)
    
        # print(f"xi {xi.shape} fhat {fhat.shape}")
        # print(f"xi {xi[0,:,:]}")
        return xi, us, fhat

    @torch.jit.unused
    def from_file(self, filename, horizon=1, dt=0.1):
        f = open(filename, 'r')
        data_states = []
        data_controls = []
        data_target = []

        states = torch.empty(0, horizon+1, self.DimX);
        controls = torch.empty(0, horizon+1, self.DimU);
        targets = torch.empty(0, horizon, self.DimF);

        # print(f"states {states}")
        for line in f:
            ls = line.split();
            if len(ls) == 0:
                if len(data_states) > 0:
                    
                    xs, us, fs = self.handle_horizon(horizon, data_states, data_controls, dt, data_target)

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
        
        # print(f"states: {states.shape}")
        return states, controls, targets
