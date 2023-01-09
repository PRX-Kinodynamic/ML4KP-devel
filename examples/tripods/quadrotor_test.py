import numpy as np 
import libpyDirtMP as prx
import matplotlib.pyplot as plt
from inspect import currentframe, getframeinfo


def distance_function(a,b):
    return prx.space_t.euclidean_2d(a,b,0,2)

if __name__ == "__main__":
    prx.set_simulation_step(0.01)

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

    lower_bounds = [0,-10.]
    upper_bounds = [100, 10.]
    ss.set_bounds(lower_bounds,upper_bounds)

    ss.copy_point_from_vector(start_state,[50.,0.])
    ss.copy_from_point(start_state)
    ss.copy_point_from_vector(goal_state,[50.,0.])

    ctrl_pt = cs.make_point()
    u_goal = cs.make_point()
    traj = [[50.,0.]]

    print(getframeinfo(currentframe()).filename, getframeinfo(currentframe()).lineno)
    # plant.linearize(goal_state,u_goal)
    Q = prx.matrix.Identity(2, 2)
    v_goal = prx.vector.Zero(2)
    v_goal[0] = 50.
    R = prx.matrix.Identity(1, 1)
    
    print(getframeinfo(currentframe()).filename, getframeinfo(currentframe()).lineno)

    lqr = prx.lqr(plant,Q,R,"LQR")
    lqr.set_goal(v_goal)
    lqr.compute_K()
    K = lqr.get_K()
    print(K)

    # for i in range(1000):
    #     cs.copy_from_point(ctrl_pt)
    #     plant.propagate(0.01)
    #     ss.copy_to_point(end_state)
    #     traj.append(end_state.to_list())

    # traj = np.array(traj)

    # plt.figure(figsize=(8,8))
    # plt.plot(traj[:,0],traj[:,1],color='black')
    # plt.show()
