import numpy as np 
import multiprocessing as mp
from collections import defaultdict
from roadmap_utils import *
from tqdm import tqdm

np.random.seed(210896)

class AccessVertex:
    def __init__(self, pt):
        self.pt = pt

class AccessEdge:
    def __init__(self, end, cost):
        self.end = end
        self.cost = cost

class AccessGraph:
    lower = -14
    upper = 14
    def __init__(self):
        self.vertices = defaultdict(AccessVertex)
        self.edges = defaultdict(list)

        self.obstacle_map = None 
        self.map_len = 0

        self.traj_output = os.environ["DIRTMP_PATH"] + "out/trajectory.txt"

        self.sr = ScriptRunner()
        self.a_indices = []
        self.d_indices = []
    
    def set_obstacle_map(self, omap):
        self.obstacle_map = omap
        self.map_len = len(omap)
        print(self.map_len)

    def _is_collision(self,s):
        # Convert state to local co-ordinates. 
        # (-14,14) in world -> (0,0) in local
        # (14,-14) in world -> (self.map_len - 1, self.map_len - 1) in local
        x = int((s[0] - self.lower) / (self.upper - self.lower) * (self.map_len - 1))
        y = int((s[1] - self.lower) / (self.upper - self.lower) * (self.map_len - 1))
        if x < 0 or x >= self.map_len:
            return True
        if y < 0 or y >= self.map_len:
            return True
        return self.obstacle_map[x][y] == 0
    
    def _is_valid(self, s):
        if s[0] < self.lower or s[0] > self.upper:
            return False
        if s[1] < self.lower or s[1] > self.upper:
            return False
        return not self._is_collision(s)
    
    def _goal_check(self, s, g):
        return np.linalg.norm(s[:3] - g[:3]) < 0.5
    
    def _check_connected(self, v1, v2):
        # Run DFS on the graph to check if there is a path from v1 to v2.
        visited = set()
        stack = [v1]
        while len(stack) > 0:
            v = stack.pop()
            if v == v2:
                return True
            visited.add(v)
            for e in self.edges[v]:
                if e.end not in visited:
                    stack.append(e.end)
        return False

    
    def get_a_indices(self,s):
        self.a_indices = []
        for i in self.vertices:
            self.sr.run(s, self.vertices[i].pt)
            traj = np.loadtxt(self.traj_output, delimiter=',')
            # If traj is only one state long, reshape.
            if len(traj.shape) == 1:
                traj = traj.reshape(1, -1)
            # Check if final state is close to goal state and 
            # if there is no collision along the trajectory.
            if self._goal_check(traj[-1], self.vertices[i].pt) and np.all([not self._is_collision(x) for x in traj]):
                self.a_indices.append(i)

    def get_d_indices(self,s):
        self.d_indices = []
        for i in self.vertices:
            self.sr.run(self.vertices[i].pt, s)
            traj = np.loadtxt(self.traj_output, delimiter=',')
            # If traj is only one state long, reshape.
            if len(traj.shape) == 1:
                traj = traj.reshape(1, -1)# Check if final state is close to goal state and
            # if there is no collision along the trajectory.
            if self._goal_check(traj[-1], s) and np.all([not self._is_collision(s) for s in traj]):
                self.d_indices.append(i)

    def get_indices(self,s):
        self.get_a_indices(s)
        self.get_d_indices(s)
    
    def build_graph(self, max_failures=50):
        current_failures = 0
        current_iters = 0
        s = np.zeros((7,))
        pbar = tqdm(total = max_failures)
        while current_failures < max_failures:
            print("Current failures: ", current_failures)
            sample_new_state = False
            while not sample_new_state:
                s[:2] = np.random.uniform(self.lower, self.upper, 2)
                s[2]  = np.random.uniform(-np.pi, np.pi)
                sample_new_state = self._is_valid(s)

            # Call get indices
            self.get_indices(s)

            changed = False

            if (len(self.a_indices) == 0) or (len(self.d_indices) == 0):
                # Add vertex
                self.vertices[len(self.vertices)] = AccessVertex(np.copy(s))
                changed = True

                # Add edges
                for i in self.a_indices:
                    self.edges[len(self.vertices) - 1].append(AccessEdge(i, np.linalg.norm(s[:3] - self.vertices[i].pt[:3])))
                for i in self.d_indices:
                    self.edges[i].append(AccessEdge(len(self.vertices) - 1, np.linalg.norm(s[:3] - self.vertices[i].pt[:3])))
            
            else:
                # Check if there is a path between any of the a_indices and d_indices.
                vertex_created = False
                for d in self.d_indices:
                    for a in self.a_indices:
                        if a != d and not self._check_connected(d,a):
                            # Add vertex
                            if not vertex_created:
                                self.vertices[len(self.vertices)] = AccessVertex(np.copy(s))
                                vertex_created = True
                                changed = True
                            # Add edge from d to new vertex
                            self.edges[d].append(AccessEdge(len(self.vertices) - 1, np.linalg.norm(s[:3] - self.vertices[d].pt[:3])))
                            # Add edge from new vertex to a
                            self.edges[len(self.vertices) - 1].append(AccessEdge(a, np.linalg.norm(s[:3] - self.vertices[a].pt[:3])))
            
            if not changed:
                current_failures += 1
                pbar.update(1)
            
        pbar.close()
    
    def add_start(self,s):
        self.get_a_indices(s)
        # Add vertex
        self.vertices[len(self.vertices)] = AccessVertex(np.copy(s))
        for i in self.a_indices:
            self.edges[len(self.vertices) - 1].append(AccessEdge(i, np.linalg.norm(s[:3] - self.vertices[i].pt[:3])))
    
    def add_goal(self,g):
        self.get_d_indices(g)
        # Add vertex
        self.vertices[len(self.vertices)] = AccessVertex(np.copy(g))
        for i in self.d_indices:
            self.edges[i].append(AccessEdge(len(self.vertices) - 1, np.linalg.norm(g[:3] - self.vertices[i].pt[:3])))
    
    def write_graph(self):  
        # Write to file
        with open(os.environ["DIRTMP_PATH"] + "out/vertices.txt", "w") as f:
            for i in self.vertices:
                f.write(str(i) + "," + str(self.vertices[i].pt[0]) + "," + str(self.vertices[i].pt[1]) + "," + str(self.vertices[i].pt[2]) + "\n")
        with open(os.environ["DIRTMP_PATH"] + "out/edges.txt", "w") as f:
            for i in self.edges:
                for e in self.edges[i]:
                    f.write(str(i)+","+str(e.end)+","+str(e.cost)+"\n")

if __name__ == "__main__":
    ag = AccessGraph()
    fname = os.environ["DIRTMP_PATH"] + "out/occupancy_map.txt"
    with open(fname, "r") as f:
        for l in f:
            last_col = len(l.split(",")) - 1
            break
    map_img = np.genfromtxt(fname, delimiter=',', dtype=None, usecols=range(0,last_col))
    ag.set_obstacle_map(map_img)
    ag.build_graph()

    start = np.array([-12.5, 10., -1.57, 0.0, 0.0, 0.0, 0.0])
    ag.add_start(start)
    goal  = np.array([12.5, 10., 1.57, 0.0, 0.0, 0.0, 0.0])
    ag.add_goal(goal)
    ag.write_graph()

    # # Write vertices to file
    # with open(os.environ["DIRTMP_PATH"] + "out/vertices.txt", "w") as f:
    #     for i in ag.vertices:
    #         f.write("{},{},{},{}\n".format(i, ag.vertices[i].pt[0], ag.vertices[i].pt[1], ag.vertices[i].pt[2]))
    
    # # Write edges to file
    # with open(os.environ["DIRTMP_PATH"] + "out/edges.txt", "w") as f:
    #     for i in ag.edges:
    #         for e in ag.edges[i]:
    #             f.write("{},{},{}\n".format(i, e.end, e.cost))
            
