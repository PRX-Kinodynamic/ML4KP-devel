import gym, torch, yaml, random, numpy as np, matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from gym import core, spaces

class Box:
    def __init__(self,geometry):
        self.center = geometry["config"]["position"]
        self.dims = geometry["collision_geometry"]["dims"]
        self.endpoints = self.get_endpoints()
        self.polygon = Polygon(self.endpoints)   
  
    def get_endpoints(self):
        p1 = [self.center[0]-0.5*self.dims[0],self.center[1]-0.5*self.dims[1]]
        p2 = [self.center[0]+0.5*self.dims[0],self.center[1]-0.5*self.dims[1]]
        p3 = [self.center[0]+0.5*self.dims[0],self.center[1]+0.5*self.dims[1]]
        p4 = [self.center[0]-0.5*self.dims[0],self.center[1]+0.5*self.dims[1]]
        points = np.array([p1,p2,p3,p4])
        return points
    
    def plot(self):
        xs, ys = self.polygon.exterior.xy    
        plt.fill(xs, ys, alpha=1.0, fc='r', ec='none')

class YAML:
    def __init__(self, yaml_file = "./YAML/passage.yaml"):
        self.polygons = []
        with open(yaml_file) as f:
            raw_yaml = yaml.load(f)
        geometries = raw_yaml["environment"]["geometries"]
        for geom in geometries:
            box = Box(geom)
            self.polygons.append(box.polygon)
        self.body = None
    
    def get_vehicle_body(self, s):
        X_DIM = 0.9
        Y_DIM = 0.6
        (x1, y1) = s[0]+X_DIM/2*np.cos(s[2])-Y_DIM/2*np.sin(s[2]), s[1]+X_DIM/2*np.sin(s[2])+Y_DIM/2*np.cos(s[2])
        (x2, y2) = s[0]+X_DIM/2*np.cos(s[2])+Y_DIM/2*np.sin(s[2]), s[1]+X_DIM/2*np.sin(s[2])-Y_DIM/2*np.cos(s[2])
        (x3, y3) = s[0]-X_DIM/2*np.cos(s[2])+Y_DIM/2*np.sin(s[2]), s[1]-X_DIM/2*np.sin(s[2])-Y_DIM/2*np.cos(s[2])
        (x4, y4) = s[0]-X_DIM/2*np.cos(s[2])-Y_DIM/2*np.sin(s[2]), s[1]-X_DIM/2*np.sin(s[2])+Y_DIM/2*np.cos(s[2])
        return Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
        
    def valid_state(self, s):
        self.body = self.get_vehicle_body(s)
        for p in self.polygons:
            # if p.interior.distance(self.body) < 0.01:
            if p.intersects(self.body):
                return False
        return True
        
    def plot_env(self):
        for p in self.polygons:
            xs, ys = p.exterior.xy    
            plt.fill(xs, ys, alpha=1.0, fc='r', ec='none')
        if self.body is not None:
            xs, ys = self.body.exterior.xy    
            plt.fill(xs, ys, alpha=1.0, fc='magenta', ec='none')

    def plot_traj(self, traj):
        self.body = self.get_vehicle_body(traj[-1])
        self.body = None
        self.plot_env()
        traj = np.vstack(traj)
        plt.plot(traj[:,0],traj[:,1], color='black')

RANDOM_SEED = 42
# torchscript_path = './Theta_Deterministic.pt'
torchscript_path = './Theta_Deterministic_eps_0_1.pt'
actor_model = torch.jit.load(torchscript_path)
actor_model.eval()
def random_seed(seed_value, use_cuda):
    np.random.seed(seed_value)  
    torch.manual_seed(seed_value)  
    random.seed(seed_value)
    if use_cuda:
        torch.cuda.manual_seed(seed_value)
        torch.cuda.manual_seed_all(seed_value)  
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
random_seed(RANDOM_SEED, True)

class SecondOrderDiffDriveGoalEnv(gym.Env):
    dt = 0.1
    prop_steps = 10
    eps = 0.5
    limits = 10.0
    # eps = 0.5

    yicrL = -.3
    yicrR =  .3
    divisor = 1./(yicrL - yicrR)

    fixed_start = False
    sparse_reward = True

    def __init__(self,max_steps=20, yaml_file = "/home/kushal/dirtmp/resources/inputs/environments/passage.yaml"):
        self.viewer = None
        self.s_high = np.array([self.limits,self.limits,np.pi,0.7,0.7])
        self.s_low  = -self.s_high

        self.u_high = np.array([1.0, 1.0, 1.0])
        self.u_low  = -self.u_high

        self.act_high = np.array([10, 10, np.pi])
        self.act_low  = -self.act_high

        self.s_dims = self.s_high.shape[0]
        self.u_dims = self.u_high.shape[0]

        self.actor_model = actor_model.to('cpu')

        self.env = YAML(yaml_file=yaml_file)
        self.traj = []

        self.observation_space = spaces.Dict({
            'observation': spaces.Box(low=self.s_low,high=self.s_high,dtype=np.float32),
            'achieved_goal' : spaces.Box(low=self.s_low[:3],high=self.s_high[:3],dtype=np.float32),
            'desired_goal': spaces.Box(low=self.s_low[:3],high=self.s_high[:3],dtype=np.float32)
        })    

        self.action_space = spaces.Box(low=self.u_low,high=self.u_high,dtype=np.float32)
        self.max_steps = max_steps
        self.reset()

    def _get_obs(self):
        return {
            'observation': np.copy(self.state),
            'achieved_goal': np.copy(self.state[:3]),
            'desired_goal' : np.copy(self.goal[:3])
        }

    def reset(self, start = None, goal = None):
        self.steps = 0
        self.state = None
        self.goal = None
        
        while self.state is None or not self.env.valid_state(self.state):
            self.state = np.zeros((self.s_dims,))
            for i in range(self.s_dims):
                self.state[i] = np.random.uniform(self.s_low[i],self.s_high[i])
        self.state[3] = self.state[4] = 0.0

        while self.goal is None or not self.env.valid_state(self.goal):
            self.goal  = np.zeros((self.s_dims,))
            for i in range(self.s_dims):
                self.goal[i]  = np.random.uniform(self.s_low[i],self.s_high[i])
        self.goal[3] = self.goal[4] = 0.0

        if start is not None:
            self.state = np.array(start)
        if goal is not None:
            self.goal = np.array(goal)

        self.traj = [list(self.state)]
        return self._get_obs()

    def _plot(self):
        start, goal = self.traj[0], self.traj[-1]
        start_region = plt.Circle((start[0], start[1]),0.2,color='green')
        plt.gca().add_patch(start_region)

        # goal_region = plt.Circle((goal[0], goal[1]),0.2,color='magenta')
        # plt.gca().add_patch(goal_region)

        goal = self.goal
        goal_region = plt.Circle((goal[0], goal[1]),0.2,color='red')
        plt.gca().add_patch(goal_region)

        self.env.plot_traj(self.traj)

    def _enforce_bounds(self,s):
        for i in range(self.s_dims):
            if i<2: continue
            if i == 2:
                while s[i] > np.pi:
                    s[i] -= 2*np.pi
                while s[i] < -np.pi:
                    s[i] += 2*np.pi
            s[i] = np.clip(s[i],self.s_low[i],self.s_high[i])
        return s

    def _propagate_once(self,s,a):
        v_fwd =  (s[4]*self.yicrL-s[3]*self.yicrR)*self.divisor
        rot_z = (s[3] - s[4])*self.divisor

        dsdt = np.array([v_fwd*np.cos(s[2]),v_fwd*np.sin(s[2]),rot_z,a[0],a[1]])
        s += dsdt * self.dt
        s =  self._enforce_bounds(s)
        return np.copy(s)

    def _terminal(self,s,goal):
        diff = self._enforce_bounds(s-goal)
        if np.linalg.norm(diff[:3]) <= self.eps: return True, True
        if self.steps >= self.max_steps: return True, False
        return False, False

    def goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis = -1)

    def preprocess_state(self, state):
        return np.array([(state[i]-self.s_low[i])/(self.s_high[i]-self.s_low[i]) for i in range(len(state))])

    def decode_state(self, state):
        return np.array([state[i]*(self.s_high[i]-self.s_low[i])+self.s_low[i] for i in range(len(state))])

    def decode_state_(self, state):
        return np.array([state[i]*(self.s_high[i]) for i in range(len(state))])

    def compute_reward(self, achieved_goal, goal,info):
        rewards = []
        for i in range(achieved_goal.shape[0]):
            diff = (achieved_goal[i]-goal[i])[:3]
            while diff[2] > np.pi:
                diff[2] -= 2*np.pi
            while diff[2] < -np.pi:
                diff[2] += 2*np.pi
            if info[i]['reward'] == -1.0 or not self.env.valid_state(achieved_goal[i]):
                # print((achieved_goal[i]))
                rewards += [-1.0]
            elif np.linalg.norm(diff) < self.eps:
                rewards += [1.0]
            else:
                rewards += [-0.01]
        return rewards

    def _propagate(self, s, lg):
        reward = -0.01
        traj = []
        for steps in range(50):
            action = self.actor_model(torch.Tensor([list(s)+list(lg)]))[0].detach().cpu()
            for i in range(2):
                action[i] = np.clip(action[i],-1,1)
                action[i] = -0.2 + 0.4 * (action[i]+1)/2

            for i in range(self.prop_steps):
                s = self._propagate_once(np.copy(s), np.copy(action))
                if not self.env.valid_state(s):
                    return self.steps>=self.max_steps, False, -1.0
            traj.append(list(s))
            
            diff = s[:3]-lg
            while diff[2] > np.pi:
                diff[2] -= 2*np.pi
            while diff[2] < -np.pi:
                diff[2] += 2*np.pi
            if np.linalg.norm(diff) < self.eps:
                break
        for t in traj:
            self.traj.append(t)
        self.state = s
        diff = self._enforce_bounds(s-self.goal)[:3]
        if np.linalg.norm(diff) < self.eps:
            return True, True, 1.0

        if self.steps>=self.max_steps: 
            return True, False, -0.01
            
        return False, False, -0.01#-(np.linalg.norm(np.array(diff)))*0.1

    def step(self,a):
        s = self.state
        u = np.copy(a)
        for i in range(self.u_dims):
            if i==2:
                while u[i]>self.u_high[i]:
                    u[i] -=2*self.u_high[i]
                while u[i]<self.u_low[i]:
                    u[i] +=2*self.u_high[i]
            else:
                u[i] = np.clip(a[i],self.u_low[i],self.u_high[i])
            u[i] = (u[i]+1)/2.0*(self.act_high[i]-self.act_low[i]) + self.act_low[i]

        terminal, success, reward = self._propagate(s, u)

        self.traj.append(list(self.state))
        self.steps += 1

        return (self._get_obs(),reward,terminal,{'is_success':success, 'obs': self.state, 'reward': reward})

class PropPenalty(SecondOrderDiffDriveGoalEnv):
    def compute_reward(self, achieved_goal, goal,info):
        rewards = []
        for i in range(achieved_goal.shape[0]):
            diff = (achieved_goal[i]-goal[i])[:3]
            while diff[2] > np.pi:
                diff[2] -= 2*np.pi
            while diff[2] < -np.pi:
                diff[2] += 2*np.pi
            if not self.env.valid_state(achieved_goal[i]):
                rewards += [info[i]['reward']]
            elif np.linalg.norm(diff) < self.eps:
                rewards += [0.0]
            else:
                rewards += [info[i]['reward']]
        return rewards

    def _propagate(self, s, lg):
        reward = 0.0
        traj = []
        for steps in range(30):
            reward += -0.1
            action = self.actor_model(torch.Tensor([list(s)+list(lg)]))[0].detach().cpu()
            for i in range(2):
                action[i] = np.clip(action[i],-1,1)
                action[i] = -0.2 + 0.4 * (action[i]+1)/2

            for i in range(self.prop_steps):
                s = self._propagate_once(np.copy(s), np.copy(action))
                if not self.env.valid_state(s):
                    return self.steps>=self.max_steps, False, -3.0
            traj.append(list(s))
            
            diff = s[:3]-lg
            while diff[2] > np.pi:
                diff[2] -= 2*np.pi
            while diff[2] < -np.pi:
                diff[2] += 2*np.pi
            if np.linalg.norm(diff) < 0.1:
                break
        for t in traj:
            self.traj.append(t)
        self.state = s
        diff = self._enforce_bounds(s-self.goal)[:3]
        if np.linalg.norm(diff) < self.eps:
            return True, True, 0.0

        if self.steps>=self.max_steps: 
            return True, False, reward
            
        return False, False, reward#-(np.linalg.norm(np.array(diff)))*0.1

