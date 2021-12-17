import sys
import math
import torch
import random
import numpy as np 
import libpyDirtMP as prx


def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
    for i in range(len(space_point)):
        pt[i] = pt[i] + step_inc
        if pt[i] <= upper_bounds[i]:
            return True
        pt[i] = lower_bounds[i]
    return False;

bn = 1
ctrl_input = None

def init_ctrl_input(plant_name):
    global ctrl_input, bn

    if plant_name == "pendulum":
        ctrl_input = torch.zeros(bn,4)


def get_control(plant_name, current, goal_state):
    global ctrl_input, bn
    ctrl = None
    if plant_name == "pendulum":
        for i in range(bn):
            ctrl_input[i,0] = current[0]
            ctrl_input[i,1] = current[1]
            ctrl_input[i,2] = goal_state[0]
            ctrl_input[i,3] = goal_state[1]
        with torch.no_grad():
            controller_out = controller(ctrl_input).cpu()
            # print("controller_out:", controller_out)
        ctrl_output_mean = 0
        for i in range(bn):
            ctrl_output_mean += 1.0*controller_out[i].item()#/bn
        ctrl_output_mean = ctrl_output_mean / bn
        ctrl = np.array([-0.6371781908344007+ ((ctrl_output_mean + 1.)*0.6371781908344007)], dtype=np.float64)
    else:
        print("get_control for plant:", plant_name, "not implemented!")
        exit(-1)
    return ctrl

if __name__ == "__main__":
    params = prx.param_loader("examples/tripods/lc_roa.yaml", sys.argv);
    path_to_model = prx.lib_path + params["/plant/controller_path"].as_string()
    controller = torch.load(path_to_model)
    controller.eval()

    params.print()

    prx.set_simulation_step(params["simulation_step"].as_float())
    prx.init_random(params["random_seed"].as_int())
    torch.manual_seed(params["random_seed"].as_int())

    # Could be done directly from params, calling the getter to show that is there...
    simulation_step = prx.get_simulation_step()
    # Safety check
    print("simulation_step:", simulation_step)

    obstacles = prx.load_obstacles(params["environment"].as_string())
    obstacle_list = obstacles.objects
    obstacle_names = obstacles.names

    plant_name = params["/plant/name"].as_string()
    plant_path = params["/plant/path"].as_string()
    plant = prx.system_factory.create_system(plant_name, plant_path);
    # plant = prx.pendulum(plant_name, plant_path)
    # prx_assert(plant != nullptr, "Plant is nullptr!");
    if plant == None: 
        print("Error: plant not found!")
        exit(-1)

    wm = prx.world_model([plant], obstacle_list)
    wm.create_context("context", [plant_name], obstacle_names)
    context = wm.get_context("context")

    ss = context.system_group.get_state_space()
    cs = context.system_group.get_control_space()

    lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
    upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
    ss.set_bounds(lower_bounds, upper_bounds)

    start_state = ss.make_point()
    goal_state  = ss.make_point()
    current     = ss.make_point()
    end_state   = ss.make_point()

    ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
    ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
    ss.copy_from_point(start_state)
    
    roa_file_name = prx.lib_path + "out/lc_" + plant_name + "_roa.txt"; 
    print("ROA file:", roa_file_name)
    fout_roa = open(roa_file_name, "w")

    trajs_file_name = prx.lib_path + "out/lc_" + plant_name + "_trajs.txt"; 
    print("Trajs file:", trajs_file_name)
    fout_trajs = open(trajs_file_name, "w")

    traj_id = 0;
    rad = params["goal_region_radius"].as_float();

    # ctrl_input = torch.zeros(1,4)
    init_ctrl_input(plant_name)

    ss_dim = ss.get_dimension()

    total_states = 1
    step_inc = params["state_increment"].as_float();

    starting_lower_bound = params["/plant/starting_lower_bound"].as_float_vector();
    ending_upper_bound = params["/plant/ending_upper_bound"].as_float_vector();
    pt = ss.make_point();
    ss.copy_point_from_vector(pt, starting_lower_bound);
    print("first pt:", pt );

    for l,u in zip(starting_lower_bound, ending_upper_bound):
        total_states *= 1. + math.floor((u - l) / step_inc);
    print("Total states:", total_states )

    traj = prx.trajectory(ss)
    next_theta = starting_lower_bound[0]
    next_theta_dot = starting_lower_bound[1]

    def distance_function(a, b):
        return prx.space_t.euclidean_2d(a, b, 0, ss_dim);
    
    def compute_traj(state):
        global next_theta, next_theta_dot, traj
        checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())
        ss.copy_from_point(state)
        ss.copy_point(end_state, state)

        traj.clear()
        traj.copy_onto_back(ss)
        # current = state.to_list()
        while True:
            ctrl = get_control(plant_name, end_state, goal_state)
            cs.copy_from_vector(ctrl)

            cs.enforce_bounds()
            plant.propagate(simulation_step)
            ss.copy_to_point(end_state);
            traj.copy_onto_back(ss)

            if checker.check():
                break

        ss.copy_to_point(end_state);
        line = str(state) + " "
        reached = 0
        # reached = distance_function(end_state, goal_state)
        if distance_function(end_state, goal_state) <= rad: reached = 1
        line += str(reached) + "\n"

        fout_roa.write(line)

        if (next_theta <= state[0] and next_theta_dot <= state[1]):
            sprev = traj[0][0]
            for s in traj:
                if math.fabs(sprev - s[0]) > 1:
                    fout_trajs.write("\n")

                fout_trajs.write(str(s) + "\n")
                sprev = s[0]
            fout_trajs.write("\n")
            if next_theta + 0.3 > ending_upper_bound[0]:
                next_theta_dot += 0.5
                next_theta = starting_lower_bound[0]
            else:
                next_theta += 0.3
            # print("Next:", next_theta, next_theta_dot, " state:", state)



    pt = ss.make_point()
    ss.copy_point_from_vector(pt, lower_bounds)

    while True:
        # print(pt)
        compute_traj(pt)
        traj_id += 1
        if not state_increment(pt, step_inc, lower_bounds, upper_bounds):
            break
    fout_trajs.close()
    print("Finished!")
