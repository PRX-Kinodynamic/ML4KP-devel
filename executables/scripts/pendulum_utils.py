import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from functools import reduce
# @torch.jit.script
class Pendulum():
    def __init__(self):
        self.DimX = 1      # thd
        self.DimU = 1      # u
        self.DimF = 1      # accel

    @torch.jit.unused
    def fwd_prop(self, horizon, vt, ft, gt, dt):
        vt1 = vt
        for _ in range(horizon):
            vt1 = vt1 + (ft+gt) * dt

        return vt1

    def handle_horizon(self, horizon, states, controls, gravity, dt):

        # us = torch.stack(controls[:-horizon])
        xi = torch.empty(len(states)-horizon, self.DimX, horizon);
        us = torch.empty(len(controls)-horizon, self.DimU, horizon);
        fhat = torch.empty(len(states)-horizon, self.DimF, horizon);
        gr = torch.empty(len(states)-horizon, self.DimX, horizon);
        # us = torch.Tensor();
        # xi = torch.Tensor();
        xH = torch.Tensor();
        # gr = torch.Tensor();
        # print(len(states), len(controls))
        # print("states", states[:10])
        for idx in range(horizon):
            up  = torch.stack(controls[idx:-horizon+idx])
            xip = torch.stack(  states[idx:-horizon+idx])
            g   = torch.stack( gravity[idx:-horizon+idx])

            us[:,:,idx] = up
            xi[:,:,idx] = xip

            gr[:,:,idx] = g;

        xH = torch.stack(states[horizon:])

        gi = torch.zeros_like(gr[:,:,0]);
        fhat = torch.Tensor();
        x0 = xi[:,:,0]
        for idx in range(horizon-1):
            xT = xi[:,:,idx+1]
            xT0 = (xT - x0) / dt
            gi = gi + gr[:,:,idx];
            fhat = torch.hstack( (fhat, xT0 - gi));
            # print("gi", gi[:10])
        xT0 = (xH - x0) / dt
        gi = gi + gr[:,(horizon-1):horizon];
        fhat = torch.hstack( (fhat, xT0 - gi));
        # print("fhat", fhat.shape, fhat[:10])
        # xH0 = (xH - xi[:,0:1]) / dt
        # gsum = []
        # for idx in range(len(gravity) - horizon):
        #     s = torch.sum( torch.Tensor(gravity[idx:idx+horizon]));
        #     # print(s)
        #     gsum.append(s)
        #     # torch.cat( (gsum, , 0 )
        # gsum = torch.vstack(gsum)
        # fhat = xH0 - gsum;

        # exit(-1)

        return xi, us, fhat

    @torch.jit.unused
    def from_file(self, filename, horizon=1, dt=0.01):
        f = open(filename, 'r')
        data_states = []
        data_controls = []
        data_target = []
        data_g = []

        states = torch.empty(0, self.DimX, horizon);
        controls = torch.empty(0, self.DimU, horizon);
        targets = torch.empty(0, self.DimF, horizon);
        # gt = torch.Tensor();

        for line in f:
            ls = line.split();
            if len(ls) == 0:
                if len(data_states) > 0:
                    xs, us, fs = self.handle_horizon(horizon, data_states, data_controls, data_g, dt)

                    # print(xs.shape, us.shape, fs.shape)
                    # data_states_tensor = torch.stack(data_states)
                    # data_controls_tensor = torch.stack(data_controls)
                    # data_target_tensor = torch.stack(data_target)
                    # print(data_target_tensor[:10])
                    # data_g_tensor = torch.stack(data_target)

                    states = torch.vstack((states, xs))
                    controls = torch.vstack((controls, us))
                    targets = torch.vstack((targets, fs))
                    # print(targets[:10])
                    # exit(1)
                    # gt = torch.cat((targets, fs), 0)

                    data_states = []
                    data_controls = []
                    data_target = []
                    data_g = []
            else:
                thd = float(ls[1])
                u = float(ls[2])
                g = float(ls[4]) # gravity
                ft = float(ls[5]) # accel

                thd = torch.Tensor([thd])
                u = torch.Tensor([u])
                ft = torch.Tensor([ft]) # accel
                g = torch.Tensor([g]) # gravity

                data_states.append(thd)
                data_controls.append(u)
                data_target.append(ft)
                data_g.append(g)

        print("states: ", states[:10])

        return states, controls, targets