import numpy as np 
import libpyDirtMP as prx
import matplotlib.pyplot as plt
from inspect import currentframe, getframeinfo


def distance_function(a,b):
    return prx.space_t.euclidean_2d(a,b,0,2)

max_height = 10
start_height = 0.00
start_vel = -20.00
time = 5
sim_step = 0.01

if __name__ == "__main__":
    prx.set_simulation_step(sim_step)

    plant_name = "1D_Quadrotor"
    plant_path = "1D_Quadrotor"
    plant = prx.system_factory.create_system(plant_name, plant_path);

    if plant == None:
        print("plant is None")
        exit(-1)
    
    wm = prx.world_model([plant],[])
    wm.create_context("context", [plant_name], [])
    context = wm.get_context("context")

    ss = context.system_group.get_state_space()
    cs = context.system_group.get_control_space()

    start_state = ss.make_point()
    goal_state  = ss.make_point()
    current     = ss.make_point()
    end_state   = ss.make_point()

    # lower_bounds = [0,-200.]
    # upper_bounds = [200, 20.]
    lower_bounds = [0,-20.]
    upper_bounds = [20, 2.]
    ss.set_bounds(lower_bounds,upper_bounds)
    cs.set_bounds([0.],[2.])

    ss.copy_point_from_vector(start_state,[start_height,start_vel])
    ss.copy_from_point(start_state)
    ss.enforce_bounds()
    ss.copy_point_from_vector(goal_state,[max_height,0.])

    ctrl_pt = cs.make_point()
    u_goal = cs.make_point()
    traj = [[start_height,start_vel]]

    # plant.linearize(goal_state,u_goal)
    Q = prx.matrix.Identity(2, 2)
    v_goal = prx.vector.Zero(2)
    v_goal[0] = max_height
    R = prx.matrix.Identity(1, 1)
    
    switch = False
    ss.copy_to_point(end_state)
    for i in range(int(time*1./sim_step)+1):
        if not switch and end_state[0] < 0.25 * max_height:
            lqr = prx.lqr(plant,Q,R,"LQR")
            lqr.set_goal(v_goal)
            lqr.compute_K()
            lqr.compute_controls()
            cs.enforce_bounds()
            switch = True
        if switch and end_state[0] > max_height:
            cs.copy_from_point(ctrl_pt)
            switch = False
        plant.propagate(0.01)
        ss.copy_to_point(end_state)
        traj.append(end_state.to_list())
        

    traj = np.array(traj)
    print(traj[-1])

    plt.figure(figsize=(8,8))
    plt.xlim(-0.1,20.1)
    plt.ylim(-20.1,2.1)
    plt.plot(traj[:,0],traj[:,1],color='black')
    plt.scatter(traj[0,0],traj[0,1],color='green')
    plt.scatter(traj[-1,0],traj[-1,1],color='red')
    plt.show()
