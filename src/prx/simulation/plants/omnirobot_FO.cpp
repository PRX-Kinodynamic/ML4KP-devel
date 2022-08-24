#include "prx/simulation/plants/omnirobot_FO.hpp"

namespace prx
{

	omnirobot_FO_t::omnirobot_FO_t(const std::string& path) : plant_t(path), D(4,3), Dp(3,4)//, mu(4)
	{
		x=y=theta=0;
		state_memory = {&x,&y,&theta};
		state_space = new space_t("EER",state_memory,"omni_state");
		state_space->set_bounds(lower_bound,upper_bound);

		x_d=y_d=theta_d=0;
		derivative_memory= {&x_d,&y_d,&theta_d};
		derivative_space = new space_t("EEE",derivative_memory,"omni_deriv");

		w1=w2=w3=w4=0;
		control_memory = {&w1,&w2,&w3,&w4};
		input_control_space = new space_t("EEEE",control_memory,"omni_ctrl");
		input_control_space->set_bounds({-200, -200, -200, -200},{200, 200, 200, 200});

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.23,.23,.1});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x000000");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		geometries["wheel_1"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["wheel_1"]->initialize_geometry({.05,.013});
		geometries["wheel_1"]->generate_collision_geometry();
		geometries["wheel_1"]->set_visualization_color("0xff0000");
		configurations["wheel_1"]= std::make_shared<transform_t>();
		configurations["wheel_1"]->setIdentity();


		geometries["wheel_2"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["wheel_2"]->initialize_geometry({.05,.013});
		geometries["wheel_2"]->generate_collision_geometry();
		geometries["wheel_2"]->set_visualization_color("0x0000ff");
		configurations["wheel_2"]= std::make_shared<transform_t>();
		configurations["wheel_2"]->setIdentity();

		geometries["wheel_3"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["wheel_3"]->initialize_geometry({.05,.013});
		geometries["wheel_3"]->generate_collision_geometry();
		geometries["wheel_3"]->set_visualization_color("0x00ff00");
		configurations["wheel_3"]= std::make_shared<transform_t>();
		configurations["wheel_3"]->setIdentity();

		geometries["wheel_4"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["wheel_4"]->initialize_geometry({.05,.013});
		geometries["wheel_4"]->generate_collision_geometry();
		geometries["wheel_4"]->set_visualization_color("0xf000f0");
		configurations["wheel_4"]= std::make_shared<transform_t>();
		configurations["wheel_4"]->setIdentity();

		constexpr double a1 = M_PI / 4.0; // Angle between wheels
		D << 	+std::sin(a1),  std::cos(a1), 1.,
				-std::sin(a1),  std::cos(a1), 1.,
				-std::sin(a1), -std::cos(a1), 1.,
				+std::sin(a1), -std::cos(a1), 1.;
		
		Dp <<	0.3536, -0.3536, -0.3536,  0.3536,
    			0.3536,  0.3536, -0.3536, -0.3536,
    			0.2500,  0.2500,  0.2500,  0.2500;
		// Rotations of the wheels with respect of the base
		
		// w1_tr.translation() = (vector_t(.125,0,0));
		// w2_tr.translation() = (vector_t(0,0.1375,0));
    	R = 0.125;
    	mu_0 = mu_1 = mu_2 = mu_3 = 1;
		set_integrator(integrator_t::kRK4);
		// set_integrator(integrator_t::kEULER);
		state = state_space -> make_point();
		ctrl = input_control_space -> make_point();
		parameter_memory = {&mu_0,&mu_1,&mu_2,&mu_3};
		parameter_space = new space_t("EEEE", parameter_memory, "omnirobot_FO_params");
		parameter_space->set_bounds({0, 0, 0, 0},{2, 2, 2, 2});

	}

    omnirobot_FO_t::~omnirobot_FO_t()
    {}

    void omnirobot_FO_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
	}

	void omnirobot_FO_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix());
		body->translation() = (vector_t(x,y,.05));

		// auto w1_tr = configurations["wheel_1"];
		// tr_aux.translation() = (vector_t(.125,0,0));
		// w1_tr.translation() = (vector_t(.125,0,0));
		transform_t w1_tr, w2_tr, w3_tr, w4_tr;
		w1_tr.linear() = (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()).toRotationMatrix());
		w1_tr.translation() = (vector_t(0,-0.125,0));
		w2_tr.linear() = (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()).toRotationMatrix());
		w2_tr.translation() = (vector_t(.125,0,0));
		w3_tr.linear() = (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()).toRotationMatrix());
		w3_tr.translation() = (vector_t(0,0.125,0));
		w4_tr.linear() = (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()).toRotationMatrix());
		w4_tr.translation() = (vector_t(-.125,0,0));
		(*configurations["wheel_1"]) = (*configurations["body"]) * w1_tr;
		(*configurations["wheel_2"]) = (*configurations["body"]) * w2_tr;
		(*configurations["wheel_3"]) = (*configurations["body"]) * w3_tr;
		(*configurations["wheel_4"]) = (*configurations["body"]) * w4_tr;
		// body->setIdentity();
		// // auto aux1 = w1_r;//quaternion_t(cos(theta),0,0,sin(theta)).toRotationMatrix();
		// // body->linear() = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix());
		// // body->linear() = (aux1);
		// body->linear() = (w1_r);
		// body->translation() = (vector_t(x+.125,y,.05));
    // <origin rpy="1.57 0.0 1.57" xyz="0.125 0 0.0"/>
	}

	bool omnirobot_FO_t::connect_points(space_point_t origin, space_point_t local_goal, plan_t& plan, trajectory_t& traj, double dist_limit, double tolerance)
	{
		traj.clear();
    	plan.clear();

		// std::cout << "origin: " << origin << std::endl;
		// std::cout << "local_goal: " << local_goal << std::endl;
		for (int i = 0; i < state_space -> get_dimension(); ++i)
		{
			// std::cout << state << std::endl;
			(*state)[i] = (*local_goal)[i] - (*origin)[i];
		}
		
		double norm = space_t::l2_norm(state);
		Xd[0] = (*state)[0] / norm;
		Xd[1] = (*state)[1] / norm;
		Xd[2] = (*state)[2] / norm;

		U = D * Xd;
    	// U[0] =  w1 * (1. - (mu[0] * mass * gravity * radius) / (4. * stall_torque));
		w1 = U[0] / (1. - (mu_0 * mass * gravity * radius) / (4. * stall_torque));
		w2 = U[1] / (1. - (mu_1 * mass * gravity * radius) / (4. * stall_torque));
		w3 = U[2] / (1. - (mu_2 * mass * gravity * radius) / (4. * stall_torque));
		w4 = U[3] / (1. - (mu_3 * mass * gravity * radius) / (4. * stall_torque));

		state_space -> copy_from_point(origin);
		state_space -> copy_to_point(state);
		double dist = space_t::euclidean_2d(state, local_goal);
		double d_traj = 0.0;
		double last_dist = dist + 1;
		traj.copy_onto_back(state_space);
		double dur = 0;

		// while (dist_limit > dist && dist > tolerance)
		while (tolerance < dist && d_traj < dist_limit)
		{	
			// std::cout << "state: " << state << std::endl;
			if ( dist > last_dist)
			{
				prx_warn("omnirobot_FO_t::connect_points: Not getting close to local goal!")
				return false;
			}

			propagate(simulation_step);
			state_space -> copy_to_point(state);
			last_dist = dist;
			dist = space_t::euclidean_2d(state, local_goal);
			d_traj = space_t::euclidean_2d(origin, state);
			traj.copy_onto_back(state_space);

			dur += simulation_step;
		}
		input_control_space -> copy_to_point(ctrl);
		plan.copy_onto_back(ctrl, dur);
		return true;

	}

	void omnirobot_FO_t::compute_derivative()
	{
		// std::cout << input_control_space -> print_memory(2) << std::endl;
		// U << w1, w2, w3, w4;
    	U[0] =  w1 * (1. - (mu_0 * mass * gravity * radius) / (4. * stall_torque));
    	U[1] =  w2 * (1. - (mu_1 * mass * gravity * radius) / (4. * stall_torque));
    	U[2] =  w3 * (1. - (mu_2 * mass * gravity * radius) / (4. * stall_torque));
    	U[3] =  w4 * (1. - (mu_3 * mass * gravity * radius) / (4. * stall_torque));
	
		Eigen::Matrix3d rot;
		rot << std::cos(theta - M_PI/4), -std::sin(theta - M_PI/4), 0,
			   std::sin(theta - M_PI/4),  std::cos(theta - M_PI/4), 0,
			   0, 0, 1;
		Xd = rot * Dp * U; // (3x3) * (3x4) * (4x1)
		x_d = Xd[0];
		y_d = Xd[1];
		theta_d = Xd[2]/R;

		// std::cout << std::setw(2) << std::fixed
		// std::cout << std::setprecision(3) << std::fixed
		// << "U: " << U.transpose() << "\t"
		// << "X: " << x << ", " << y << ", " << theta << "\t"
		// << "Xd: " << Xd.transpose() << std::endl;
	}
}