import sys
import math
import torch
import random
import numpy as np 
import libpyDirtMP as prx



if __name__ == "__main__":
    params = prx.param_loader("examples/tripods/ackermann_ha_roa.yaml", sys.argv);

    simulation_step = params["simulation_step"].as_float()
    prx.set_simulation_step(simulation_step)
    prx.init_random(params["random_seed"].as_int())

    obstacles = prx.load_obstacles(params["environment"].as_string())
    obstacle_list = obstacles.objects
    obstacle_names = obstacles.names

    plant_name = params["/plant/name"].as_string();
    plant_path = params["/plant/path"].as_string();
    plant = prx.system_factory.create_system(plant_name, plant_path);
    if plant == None: 
        print("Error: plant not found!")
        exit(-1)

    wm = prx.world_model([plant], obstacle_list)
    wm.create_context("context", [plant_name], obstacle_names)
    context = wm.get_context("context")

    ss = context.system_group.get_state_space();
    cs = context.system_group.get_control_space();

    lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
    upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
    ss.set_bounds(lower_bounds, upper_bounds);

    cs_lb = params["/plant/control_space_lower_bound"].as_float_vector()
    cs_up = params["/plant/control_space_upper_bound"].as_float_vector()
    cs.set_bounds(cs_lb, cs_up);

    start_state = ss.make_point();
    goal_state = ss.make_point();
    end_state   = ss.make_point()

    ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
    ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
    ss.copy_from_point(start_state)
    
    checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int());

    solution_traj = prx.trajectory(ss);

    ss_dim = ss.get_dimension();
    cs_dim = cs.get_dimension();

    ss.print_bounds();
    cs.print_bounds();
        
    u_goal = cs.make_point();

    u_goal[0] = 0;
    u_goal[1] = 1;

    plant.linearize(goal_state, u_goal);
    
    Q = prx.matrix.Identity(ss_dim, ss_dim)
    R = prx.matrix.Identity(cs_dim, cs_dim);
    q_vec = params["/plant/lqr_Q"].as_float_vector();
    
    for i in range(ss_dim):
        Q[i,i] = q_vec[i]

    lqr = prx.lqr(plant, Q, R, "LQR");
    lqr.set_goal(goal_state);
    lqr.compute_K();
    
    ss.copy_from_point(start_state);
    solution_traj.copy_onto_back(ss);
    while True:
        lqr.compute_controls();
        cs.enforce_bounds();
        plant.propagate(simulation_step);
        
        # Remove the following line when doing intensive computation!
        solution_traj.copy_onto_back(ss);

        if checker.check():
            break

    print("Last state: ", solution_traj.back(), " distance: ", prx.space_t.euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim));

    vis_group = prx.three_js_group([plant], obstacle_list)

    body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

    vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

    vis_group.add_animation(solution_traj, ss, start_state)

    params.print();
    vis_group.output_html("py_lqr_ctrl.html")

