
#include "prx/simulation/plants/pendulum.hpp"

namespace prx
{

	pendulum_t::pendulum_t(const std::string& path) : lti_t(path)
	{
		_theta1=_theta1dot=0;
		state_memory = {&_theta1,&_theta1dot};
		lti_stt_space = state_space = new space_t("RE",state_memory,"pendulum_state");
		state_space->set_bounds({-PRX_PI,-2*PRX_PI},{PRX_PI,2*PRX_PI});
		// state_space->set_bounds({-3.15,-3.15,-6,-6},{3.15,3.15,6,6});

		_tau=0;
		control_memory = {&_tau};
		lti_ctr_space = input_control_space = new space_t("E",control_memory,"Torque");
		input_control_space->set_bounds({-0.6371781908344007},{0.6371781908344007});

		_theta1dotdot=0;
		derivative_memory = {&_theta1dot,&_theta1dotdot};
		derivative_space = new space_t("EE",derivative_memory,"pendulum_deriv");


		parameter_memory = {&length,&friction,&mass,&normalize};
		parameter_space = new space_t("EEED", parameter_memory, "pendulum_params");

		const double length = 20;

		geometries["rod1"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["rod1"]->initialize_geometry({length,1,1});
		geometries["rod1"]->generate_collision_geometry();
		geometries["rod1"]->set_visualization_color("0x00ff00");
		configurations["rod1"]= std::make_shared<transform_t>();
		configurations["rod1"]->setIdentity();

		geometries["ball"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["ball"]->initialize_geometry({1.5});
		geometries["ball"]->generate_collision_geometry();
		geometries["ball"]->set_visualization_color("0x0000ff");
		configurations["ball"]= std::make_shared<transform_t>();
		configurations["ball"]->setIdentity();

		// set_integrator("rk4");
		set_integrator(integrator_t::kRK4);


		
	}

	pendulum_t::~pendulum_t()
	{

	}

	void pendulum_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);

        _theta1 = norm_angle_pi(_theta1, -PRX_PI, PRX_PI);
        state_space -> enforce_bounds();
	}

	void pendulum_t::update_configuration()
	{
        const double theta1 = _theta1 - M_PI / 2.0;

		const double length = 20;
		auto body = configurations["rod1"];
		body->setIdentity();
		Eigen::Matrix3d m;
		m =   Eigen::AngleAxisd(-theta1, Eigen::Vector3d::UnitZ())
		    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		body->linear() = (m);
		body->translation() = (vector_t((length / 2.0) * cos(theta1), 
										-(length / 2.0) * sin(theta1), 
										1.5));

		body = configurations["ball"];
		body->setIdentity();
		body->translation() = (vector_t((length) * cos(theta1),
                                        -(length) * sin(theta1),
                                        1.5));
	}

	void pendulum_t::compute_derivative()
	{
        const double theta1 = _theta1;// - M_PI / 2.0;

		compute_inertia();

        _theta1dotdot = gravity / length * std::sin(theta1) + _tau / inertia;

        // if (friction > 0)
        // {
            _theta1dotdot -= friction / inertia * _theta1dot;
        // }

	}

	bool pendulum_t::linearize()
	{
		compute_inertia();

		A.resize(2,2);
		B.resize(2,1);
		C.resize(2,2);
		D.resize(2,1);
		A << 0., 1., gravity / length, -friction / inertia;

        B << 0., 1. / inertia;

        std::cout << "A: " << A << std::endl;
    	std::cout << "B: " << B << std::endl;
        if (normalize != 0)
        {

            // self.normalization = [np.array(norm, dtype=config.np_dtype)
            //                       for norm in normalization]
            // self.inv_norm = [norm ** -1 for norm in self.normalization]
        	auto ss_ub = state_space -> get_upper_bounds();
        	auto cs_ub = input_control_space -> get_upper_bounds();
        	Tx.diagonal() << ss_ub[0], ss_ub[1];
        	Tu.diagonal() << cs_ub[0];
        	Tx_inv.diagonal() << (1.0 / ss_ub[0]), (1.0 / ss_ub[1]);
        	Tu_inv.diagonal() << (1.0 / cs_ub[0]);
        	// auto Tu = map(np.diag, self.normalization)
         //    Tx_inv, Tu_inv = map(np.diag, self.inv_norm)
        	std::cout << "Tx: " << Tx.diagonal() << std::endl;
    		std::cout << "Tu: " << Tu.diagonal() << std::endl;

        	std::cout << "Tx_inv: " << Tx_inv.diagonal() << std::endl;
    		std::cout << "Tu_inv: " << Tu_inv.diagonal() << std::endl;

            A = Tx_inv * A * Tx;
            B = Tx_inv * B * Tu;

        }
        std::cout << "A: " << A << std::endl;
    	std::cout << "B: " << B << std::endl;
PRX_DEBUG_PRINT
        // discretize();
PRX_DEBUG_PRINT

        // if self.normalization is not None:
        //     Tx, Tu = map(np.diag, self.normalization)
        //     Tx_inv, Tu_inv = map(np.diag, self.inv_norm)

        //     A = np.linalg.multi_dot((Tx_inv, A, Tx))
        //     B = np.linalg.multi_dot((Tx_inv, B, Tu))

        // sys = signal.StateSpace(A, B, np.eye(2), np.zeros((2, 1)))
        // sysd = sys.to_discrete(self.dt)
        // 
        return true;
	}

}