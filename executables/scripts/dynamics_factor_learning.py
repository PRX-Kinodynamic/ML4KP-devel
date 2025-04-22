import argparse
import torch
import torch.nn as nn
import numpy as np
import copy
import scipy
from torch.utils.data import Dataset, DataLoader, random_split
from pendulum_utils import Pendulum
from mj_mushr_utils import MjMushr
import os

# from enum import Enum


@torch.jit.script
class GaussianNormalization():
    def __init__(self, A, Ainv, mu):
        # if other is not None:
        self.A = A;
        self.Ainv = Ainv;
        self.mu = mu;

    @torch.jit.unused
    def normalize_data(self, data_in):
        data = np.array(data_in).T
        mean = torch.Tensor(np.mean(data, axis=1))
        # print(data.shape)
        sigma = np.cov(data)
        # print(sigma.ndim)
        if sigma.ndim == 0:
            sigma = np.array([[sigma]])
        print("Mean", mean)
        print("Cov: ", sigma)
        V,L,Vp = np.linalg.svd(sigma)

        S = scipy.linalg.sqrtm(np.diag(L))
        Sinv = scipy.linalg.sqrtm(np.diag(1/L))
        T = V @ S

        assert np.isclose(sigma, T @ T.T).all(), "[Normalization] Decomposition failed"

        A = torch.Tensor(T)
        Ainv = torch.Tensor(Sinv @ V.T)

        data_out = (Ainv @ (data_in - mean).T).T

        self.A = A;
        self.Ainv = Ainv;
        self.mu = mean;
        return data_out

    @torch.jit.export 
    def normalize(self, x):
        return self.Ainv @ (x - self.mu)

    @torch.jit.export 
    def unnormalize(self, z):
        return self.A @ z + self.mu

@torch.jit.script
class UniformNormalization():

    def __init__(self, max_vals):
        self.max_vals = max_vals;

    @torch.jit.unused
    def normalize_data(self, inputs):
        p = torch.amax(inputs.T, dim=1, keepdim=False)
        m = torch.amin(inputs.T, dim=1, keepdim=False)

        self.max_vals = torch.fmax(torch.abs(p), torch.abs(m))

        return torch.div(inputs, self.max_vals)

    @torch.jit.export 
    def normalize(self, x):
        return torch.div(x, self.max_vals)

    @torch.jit.export 
    def unnormalize(self, z):
        return torch.mul(z, self.max_vals)

class Normalization():
    # GUASSIAN = 1
    # UNIFORM = 2

    @staticmethod
    def from_string(method):
        if method.lower() == "gaussian":
            return GaussianNormalization(None, None, None);
        elif method.lower() == "uniform":
            return UniformNormalization(None);
        raise ValueError("No normalization method")

    @staticmethod
    def copy(obj):
        print(type(obj).__name__)
        if isinstance(obj, GaussianNormalization):
            # return torch.jit.trace(obj.normalize, (torch.rand(3)))
            return GaussianNormalization(obj.A, obj.Ainv, obj.mu)
        if isinstance(obj, UniformNormalization):
            return UniformNormalization(obj.max_vals)

class CppModule(torch.nn.Module):
    def __init__(self, dataset, other):
        super(CppModule, self).__init__()
        # self.plant = dataset.plant
        self.model = copy.deepcopy(other.model)

        # self.state_normalizer = Normalization.copy(dataset.state_normalizer)
        # self.control_normalizer = Normalization.copy(dataset.control_normalizer)
        # self.target_normalizer_ = Normalization.copy(dataset.target_normalizer)
        
        self.state_normalizer = torch.jit.trace(dataset.state_normalizer.normalize, (torch.rand(dataset.plant.DimX)))
        self.control_normalizer = torch.jit.trace(dataset.control_normalizer.normalize, (torch.rand(dataset.plant.DimU)))
        self.target_unnormalizer = torch.jit.trace(dataset.target_normalizer.unnormalize, (torch.rand(dataset.plant.DimF)))
        # 
        # self.controls_method = Normalization.UNIFORM;
        # if self.states_method == "gaussian":
        #     self.states_method = Normalization.GUASSIAN;
        # elif self.states_method == "uniform":
            # self.states_method = Normalization.UNIFORM;

        # print("self.states_method", self.states_method)
    # @torch.jit.export 
    # def normalize(self, x, method : Normalization, Ainv, mu, max_vals):
        # return Ainv @ torch.sub(x, mu)
        # if method == Normalization.GUASSIAN:
        #     # z = A^-1 * (X - mu)
        #     return Normalization.gaussian_normalize(x, Ainv, mu)
        # elif method == Normalization.UNIFORM:
        #     return Normalization.uniform_normalize(x, max_vals)
        # else:
        #     raise ValueError("Method not supported: ", method)

    # @torch.jit.export 
    # def gaussian_normalize(self, x, Ainv, mu, max_vals = None):
    #     return Ainv @ (x - mu)

    # @torch.jit.export 
    # def uniform_normalize(self, x, max_vals, Ainv = None, mu = None):
    #     return torch.div(x, max_vals)


    # @torch.jit.export
    # def unnormalize(self, z, method : int, A = None, mu = None, max_vals = None):
    #     if method == 0:
    #         # X = A * z + mu
    #         return A @ z + mu
    #     elif method == 1:
    #         return torch.mul(z, max_vals)
    #     else:
    #         raise ValueError("Method not supported: ", method)

    # @torch.jit.export 
    # def normalize_input(self, xin):
    #     return self.A_in_inv @ (xin - self.in_mean)

    # @torch.jit.export 
    # def normalize_output(self, xout):
    #     return self.A_out_inv @ (xout - self.out_mean)
    
    # @torch.jit.export
    # def unnormalize_output(self, xout):
    #     return self.out_mean + self.A_out @ xout

    def forward(self, x, u):
        x_norm = self.state_normalizer(x)
        u_norm = self.control_normalizer(u)
        xu = torch.hstack(x_norm, u_norm);
        y_norm = self.model(xu)
        y = self.target_unnormalizer(y_norm) # traced unnormalized
        # y = self.target_normalizer.unnormalize(y_norm)
        return y;
        # return y_norm;
        # return x_norm;
        # return x;
        # return xu
        


# Assuming that the system is \dot{x}_{t+1} = \dot{x}_t + f(\dot{x}_t, u_t) \delta t
# Separating data into state:=\dot{x}, controls:=u and targets:=f(\dot{x}_t, u_t)
class TransitionDataset(Dataset):
    def __init__(self, plant_name, filename = "",\
         states_norm = "gaussian", controls_norm = "uniform", targets_norm = "gaussian",\
         dbg = True):
        if plant_name == "mj_mushr":
            self.plant = MjMushr()
        elif plant_name == "pendulum":
            self.plant = Pendulum()
        else:
            throw("Wrong plant ", plant);
        self.states = torch.Tensor();
        self.controls = torch.Tensor();
        self.targets = torch.Tensor();
        
        self.state_normalizer = Normalization.from_string(states_norm);
        self.control_normalizer = Normalization.from_string(controls_norm);
        self.target_normalizer = Normalization.from_string(targets_norm);

        if dbg:
            print("Normalization states method:", states_norm)
            print("Normalization controls method:", controls_norm)
            print("Normalization targets method:", targets_norm)
        
        self.dbg = dbg;
        self.dbg_dir = os.environ['DIRTMP_PATH'] + "/out/dbg/"

        assert self.states.shape[0] == self.controls.shape[0], "States and controls must be the same shape[0]"

    def __len__(self):
        return self.states.shape[0]

    def __getitem__(self, idx):
        # return self.states[idx], self.controls[idx], self.targets[idx]
        return np.hstack((self.states[idx], self.controls[idx])), self.targets[idx]

    def from_file(self, filename):
        states, controls, targets = self.plant.from_file(filename);        
        self.states = torch.cat((self.states, states), 0)
        self.controls = torch.cat((self.controls, controls), 0)
        self.targets = torch.cat((self.targets, targets), 0)

    def gaussian_normalize(self, data_in):
        # print(data_in.shape)
        data = np.array(data_in).T
        mean = torch.Tensor(np.mean(data, axis=1))
        # print(data.shape)
        sigma = np.cov(data)
        # print(sigma.ndim)
        if sigma.ndim == 0:
            sigma = np.array([[sigma]])
        # print("Mean", mean)
        # print("Cov: ", sigma)
        V,L,Vp = np.linalg.svd(sigma)

        # L is N diag ==> inv(diag(L)) is equivalent to  1/L_{ii} i={1,...,N}

        S = scipy.linalg.sqrtm(np.diag(L))
        Sinv = scipy.linalg.sqrtm(np.diag(1/L))
        T = V @ S

        assert np.isclose(sigma, T @ T.T).all(), "[Normalization] Decomposition failed"

        A = torch.Tensor(T)
        Ainv = torch.Tensor(Sinv @ V.T)

        data_out = (Ainv @ (data_in - mean).T).T

        return data_out, A, Ainv, mean

    def uniform_normalize(self, inputs):
        p = torch.amax(inputs.T, dim=1, keepdim=False)
        m = torch.amin(inputs.T, dim=1, keepdim=False)

        max_vals = torch.fmax(torch.abs(p), torch.abs(m))

        return torch.div(inputs, max_vals), max_vals

    def apply_normalization(self, method, data):
        normalized_data = A_in = A_inv = mean = max_vals = torch.Tensor([])
            
        gaussian_data, A_in, A_inv, mean = self.gaussian_normalize(data)
        uniform_data, max_vals = self.uniform_normalize(data);
        if method == Normalization.GUASSIAN:
            normalized_data = gaussian_data
        elif method == Normalization.UNIFORM:
            normalized_data = uniform_data

        assert normalized_data != None, "Problem normalizing data"

        return normalized_data, A_in, A_inv, mean, max_vals



    def normalize(self):

        # self.states, self.Ax, self.Ax_inv, self.x_mean, self.states_max_vals = self.apply_normalization(self.states_method, self.states)
        # self.controls, self.Au, self.Au_inv, self.u_mean, self.controls_max_vals = self.apply_normalization(self.controls_method, self.controls)
        # self.targets, self.Af, self.Af_inv, self.f_mean, self.targets_max_vals = self.apply_normalization(self.targets_method, self.targets)

        self.states = self.state_normalizer.normalize_data(self.states);
        self.controls = self.control_normalizer.normalize_data(self.controls);
        self.targets = self.target_normalizer.normalize_data(self.targets);

        if self.dbg:
            torch.save(self.states, self.dbg_dir + "/states.pt")
            torch.save(self.controls, self.dbg_dir + "/controls.pt")
            torch.save(self.targets, self.dbg_dir + "/targets.pt")
            print("[Dbg] ", self.dbg_dir + "/states.pt")
            print("[Dbg] ", self.dbg_dir + "/controls.pt")
            print("[Dbg] ", self.dbg_dir + "/targets.pt")
        # uniform_states, self.states_max_vals = self.uniform_normalize(self.states);
        # uniform_controls, self.controls_max_vals = self.uniform_normalize(self.controls);
        # uniform_targets, self.targets_max_vals = self.uniform_normalize(self.targets);

        # gaussian_states, self.Ax_in ,self.Ax_inv, self.x_mean = self.gaussian_normalize(self.states)
        # gaussian_controls, self.Au_in ,self.Au_inv, self.u_mean = self.gaussian_normalize(self.controls)
        # gaussian_targets, self.Af_out ,self.Af_inv, self.f_mean = self.gaussian_normalize(self.targets)



def model_to_cpp_script(plant, model, out_filename):
    modelcpp = CppModule(plant, model)
    sm = torch.jit.script(modelcpp)
    # example_weight = torch.rand(1, 1, 3, 3)
    # example_forward_input = torch.rand(1, 1, 3, 3)

    # sm = torch.jit.trace(modelcpp.forward, )
    sm.save(out_filename)
    # torch.save(sm.state_dict(), out_filename);

class MLP(nn.Module):

    def __init__(self, input_dim, hidden_sizes, output_dim, dropout_rate=0.05):
        super(MLP, self).__init__()
        
        # Create list to hold all layers
        layers = []
        
        # Input layer
        # SiLU
        # Mish
        # activation_function = nn.Mish()
        activation_function = nn.SiLU()
        # activation_function = nn.ReLU()
        # activation_function = nn.LeakyReLU()
        layers.append(nn.Linear(input_dim, hidden_sizes[0]))
        layers.append(activation_function)
        layers.append(nn.Dropout(dropout_rate))
        
        # Hidden layers
        for i in range(len(hidden_sizes)-1):
            layers.append(nn.Linear(hidden_sizes[i], hidden_sizes[i+1]))
            layers.append(activation_function)
            layers.append(nn.Dropout(dropout_rate))
        
        # Output layer
        layers.append(nn.Linear(hidden_sizes[-1], output_dim))
        # layers.append(nn.Sigmoid())  # For binary classification
        
        # Combine all layers into a sequential model
        self.model = nn.Sequential(*layers)
    # def __init__(self, input_dim=2, hidden_dim=16, output_dim=2):
    #     super(MLP, self).__init__()
    #     self.fc1 = nn.Linear(input_dim, hidden_dim)
    #     self.relu = nn.ReLU()
    #     self.fc2 = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        return self.model(x)
    #     # x shape: [batch_size, 2]
    #     x = self.fc1(x)
    #     x = self.relu(x)
    #     x = self.fc2(x)
    #     return x

class Trainer:

    def __init__(self, model, dataloader, lr=1e-4, device='cpu'):
        self.model = model.to(device)
        self.dataloader = dataloader
        self.criterion = nn.MSELoss() # euclidean loss
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr) # optimizer for learning
        self.device = device

    def train(self, epochs=20):
        self.model.train()
        for epoch in range(1, epochs+1):
            total_loss = 0.0
            for batch_inputs, batch_targets in self.dataloader:
                # Move data to the desired device
                batch_inputs = batch_inputs.to(self.device)
                batch_targets = batch_targets.to(self.device)

                # Forward
                outputs = self.model(batch_inputs)
                loss = self.criterion(outputs, batch_targets)

                # Backprop
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                # print(f"Batch Loss: {loss.item():.6f}") # just for show, you don't really need this

                total_loss += loss.item()

            avg_loss = total_loss / len(self.dataloader)
            print(f"Epoch {epoch}/{epochs}, Loss: {avg_loss:.6f}")

    def validate(self, dataloader):
        self.model.eval()
        with torch.no_grad():
            total_loss = 0.0
            for batch_inputs, batch_targets in dataloader:
                batch_inputs = batch_inputs.to(self.device)
                batch_targets = batch_targets.to(self.device)

                outputs = self.model(batch_inputs)
                loss = self.criterion(outputs, batch_targets)

                # print(f"Validate Loss: {loss.item():.6f}") # just for show, you don't really need this
                total_loss += loss.item()
            avg_loss = total_loss / len(self.dataloader)
            print(f"Avg Validate Loss: {avg_loss:.6f}") # just for show, you don't really need this

    def predict(self, data):
        # your data has to be on the same device as the model
        data = data.to(self.device)
        # you can batch as well
        self.model.eval()
        with torch.no_grad():
          return self.model(data)


if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    # argparse.add_argument('-f', '--file', help='Gait file', required=True)
    argparse.add_argument('-d', '--dir', help='Gait file', required=True)
    argparse.add_argument('-f', '--files', help='Gait file', required=True, nargs='+')
    argparse.add_argument('-v', '--val', help='Gait file', required=True)
    argparse.add_argument('-o', '--out', help='Split validation at this rate', required=True)
    argparse.add_argument('-e', '--epochs', help='Split validation at this rate', required=True)
    argparse.add_argument('-b', '--batch', help='Batch size', required=True)
    argparse.add_argument('-p', '--plant', help='plant', required=True)
    argparse.add_argument('--x_norm', help='Normalization for states', required=True)
    argparse.add_argument('--u_norm', help='Normalization for controls', required=True)
    argparse.add_argument('--f_norm', help='Normalization for targets', required=True)
    argparse.add_argument('--total_layers', help='total hidden layers', required=True)
    argparse.add_argument('--hidden_size' , help='hidden size', required=True)
    args = argparse.parse_args()

    input_dir = args.dir;
    # train_dataset = TransitionDataset(args.plant)
    states_norm = args.x_norm;
    controls_norm = args.u_norm;
    targets_norm = args.f_norm;

    train_dataset = TransitionDataset(args.plant, states_norm = states_norm, controls_norm = controls_norm, targets_norm = targets_norm)
    validation_dataset = TransitionDataset(args.plant, states_norm = states_norm, controls_norm = controls_norm, targets_norm = targets_norm, dbg = False)


    for f in args.files:
        filename = input_dir + "/" + f
        print(filename)
        train_dataset.from_file(filename);
    
    validation_dataset.from_file(input_dir + "/" + args.val);

    train_dataset.normalize();
    validation_dataset.normalize();

    # validation_dataset = TransitionDataset(args.plant,input_dir + "/" + args.val)
    # split = float(args.split);
    # validation_dataset, test_set = random_split(validation_dataset, [split, 1-split])
    # print(len(validation_dataset))
    # print(dataset[0])
    # dataset[0] # example

    # def __init__(self, input_dim=2, hidden_dim=16, output_dim=2):
    DimIn = train_dataset.plant.DimX + train_dataset.plant.DimU
    DimOut = train_dataset.plant.DimF

    hidden_sizes = int(args.total_layers) * [int(args.hidden_size)]
    model = MLP(input_dim=DimIn, hidden_sizes=hidden_sizes, output_dim=DimOut)

    batch_size = int(args.batch)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True) # for randomly sampling from the dataset and batching
    validation_dataloader = DataLoader(validation_dataset, batch_size=batch_size, shuffle=False) # for randomly sampling from the dataset and batching

    trainer = Trainer(model, train_dataloader, device) # setup the training process
    trainer.train(epochs=int(args.epochs))
    trainer.validate(validation_dataloader)

    model_to_cpp_script(train_dataset, model, args.out)

    # for x,y in test_set:
    #     print(trainer.predict(x),y)
        
