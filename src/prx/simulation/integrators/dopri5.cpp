#include "prx/simulation/integrators/dopri5.hpp"

namespace prx
{
dopri5_t::dopri5_t(space_t* state_space, space_t* derivative_space, std::function<void()> deriv_f,
                   const double initial_simulation_step, const double _atol, const double _rtol, const int _nsteps,
                   const double _max_step, const double _safety, const double _ifactor, const double _dfactor,
                   const double _beta)
  : integrator_t(state_space, derivative_space, deriv_f, initial_simulation_step)
{
  Atol = _state_space->make_point();
  Rtol = _state_space->make_point();
  start_integration_state = _state_space->make_point();
  k1 = _derivative_space->make_point();
  k2 = _derivative_space->make_point();
  k3 = _derivative_space->make_point();
  k4 = _derivative_space->make_point();
  k5 = _derivative_space->make_point();
  k6 = _derivative_space->make_point();
  k7 = _derivative_space->make_point();
  k_cache = _derivative_space->make_point();
  yn = _state_space->make_point();
  yhn = _state_space->make_point();

  set_Atol(_atol);
  set_Rtol(_rtol);
  nstep = _nsteps;
  max_step = _max_step;
  safety = _safety;
  facmax = _ifactor;
  facmin = _dfactor;
  beta = _beta;
  h_initial = initial_simulation_step;
}

dopri5_t::dopri5_t(space_t* _state_space, space_t* _derivative_space, std::function<void()> deriv_f,
                   const double initial_simulation_step, const std::vector<double> _atol,
                   const std::vector<double> _rtol, const int _nsteps, const double _max_step, const double _safety,
                   const double _ifactor, const double _dfactor, const double _beta)
  : integrator_t(_state_space, _derivative_space, deriv_f, initial_simulation_step)
{
  Atol = _state_space->make_point();
  Rtol = _state_space->make_point();
  start_integration_state = _state_space->make_point();
  k1 = _derivative_space->make_point();
  k2 = _derivative_space->make_point();
  k3 = _derivative_space->make_point();
  k4 = _derivative_space->make_point();
  k5 = _derivative_space->make_point();
  k6 = _derivative_space->make_point();
  k7 = _derivative_space->make_point();
  k_cache = _derivative_space->make_point();
  yn = _state_space->make_point();
  yhn = _state_space->make_point();

  set_Atol(_atol);
  set_Rtol(_rtol);
  nstep = _nsteps;
  max_step = _max_step;
  safety = _safety;
  facmax = _ifactor;
  facmin = _dfactor;
  beta = _beta;
  h_initial = initial_simulation_step;
}

dopri5_t::~dopri5_t()
{
}

void dopri5_t::set_Atol(double atol_sclr)
{
  prx_assert(_state_space != nullptr, "Trying to use a NULL state space!");
  set_Atol(std::vector<double>(_state_space->get_dimension(), atol_sclr));
}

void dopri5_t::set_Atol(std::vector<double> atol_vec)
{
  prx_assert(_state_space != nullptr, "Trying to use a NULL state space!");
  _state_space->copy_point_from_vector(Atol, atol_vec);
}

void dopri5_t::set_Rtol(double rtol_sclr)
{
  prx_assert(_state_space != nullptr, "Trying to use a NULL state space!");
  set_Rtol(std::vector<double>(_state_space->get_dimension(), rtol_sclr));
}

void dopri5_t::set_Rtol(std::vector<double> rtol_vec)
{
  prx_assert(_state_space != nullptr, "Trying to use a NULL state space!");
  _state_space->copy_point_from_vector(Rtol, rtol_vec);
}

double dopri5_t::error(const space_point_t& y1, const space_point_t& yh1)
{
  prx_assert(Atol != nullptr, "Atol not initialized!");
  prx_assert(Rtol != nullptr, "Rtol not initialized!");

  using std::abs;
  using std::max;
  using std::pow;

  double err = 0.0;
  double atoli, rtoli, y1i, yh1i, sci;
  for (int i = 0; i < _state_space->get_dimension(); ++i)
  {
    atoli = Atol->at(i);
    rtoli = Rtol->at(i);
    y1i = y1->at(i);
    yh1i = yh1 == nullptr ? 0.0 : yh1->at(i);
    sci = atoli + max(abs(y1i), abs(yh1i)) * rtoli;

    // printf("\terr: %.6f atoli: %.4f rtoli: %.4f y1i: %.4f yh1i: %.4f sci: %.4f\n",
    // 		err, atoli, rtoli, y1i, yh1i, sci);
    err += pow((y1i - yh1i) / sci, 2);
  }
  // std::cout << "\terr: " << std::scientific;
  // 		std::cout << err;
  return sqrt(err / float(_state_space->get_dimension()));
}

void dopri5_t::step()
{
  _state_space->copy_to_point(start_integration_state);

  // Compute k1 = f(X0, Y0)
  _compute_derivative();
  _derivative_space->copy_to_point(k1);

  if (_h == 0.0)
    initial_h();

  // Compute k2 = f(X0, Y0 + a21 * k1)
  _state_space->integrate(start_integration_state, _derivative_space, _h * a21);
  _compute_derivative();
  _derivative_space->copy_to_point(k2);

  // Compute k3 = f(X0 + C3h, Y0 + h * (a31 * k1 + a32 * k2))
  _derivative_space->copy_to_point(k_cache);
  k_cache->multiply(_h * a32);
  k_cache->add_multiply(_h * a31, k1);
  _state_space->copy_point(yn, start_integration_state);
  yn->add(k_cache);
  _state_space->copy_from_point(yn);
  _compute_derivative();
  _derivative_space->copy_to_point(k3);

  // Compute k4 = f(X0 + C4 * h, Y0 + h * (a41 * k1 + a42 * k2 + a43 * k3))
  _derivative_space->copy_to_point(k_cache);
  k_cache->multiply(_h * a43);
  k_cache->add_multiply(_h * a42, k2);
  k_cache->add_multiply(_h * a41, k1);
  _state_space->copy_point(yn, start_integration_state);
  yn->add(k_cache);
  _state_space->copy_from_point(yn);
  _compute_derivative();
  _derivative_space->copy_to_point(k4);

  // Compute k5 = f(X0 + C5 * h, Y0 + h * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4))
  _derivative_space->copy_to_point(k_cache);
  k_cache->multiply(_h * a54);
  k_cache->add_multiply(_h * a53, k3);
  k_cache->add_multiply(_h * a52, k2);
  k_cache->add_multiply(_h * a51, k1);
  _state_space->copy_point(yn, start_integration_state);
  yn->add(k_cache);
  _state_space->copy_from_point(yn);
  _compute_derivative();
  _derivative_space->copy_to_point(k5);

  // Compute k6 = f(X0 + C6 * h, Y0 + h * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5))
  _derivative_space->copy_to_point(k_cache);
  k_cache->multiply(_h * a65);
  k_cache->add_multiply(_h * a64, k4);
  k_cache->add_multiply(_h * a63, k3);
  k_cache->add_multiply(_h * a62, k2);
  k_cache->add_multiply(_h * a61, k1);
  _state_space->copy_point(yn, start_integration_state);
  yn->add(k_cache);
  _state_space->copy_from_point(yn);
  _compute_derivative();
  _derivative_space->copy_to_point(k6);

  // Compute k7 = f(X0 + C7 * h, Y0 + h * (a71 * k1 + a72 * k2 + a73 * k3 + a74 * k4 + a75 * k5 + a76 * k6))
  _derivative_space->copy_to_point(k_cache);
  k_cache->multiply(_h * a76);
  k_cache->add_multiply(_h * a75, k5);
  k_cache->add_multiply(_h * a74, k4);
  k_cache->add_multiply(_h * a73, k3);
  k_cache->add_multiply(_h * a72, k2);
  k_cache->add_multiply(_h * a71, k1);
  _state_space->copy_point(yn, start_integration_state);
  yn->add(k_cache);
  _state_space->copy_from_point(yn);
  _compute_derivative();
  _derivative_space->copy_to_point(k7);
  _derivative_space->copy_to_point(k_cache);

  int n = _state_space->get_dimension();
  // for (int i = 0; i < n; ++i)
  // {
  // 	_derivative_space -> at(i) = start_integration_state -> at(i) +
  // 					   			h * ( a71 * k1 -> at(i) +
  // 									  a72 * k2 -> at(i) +
  // 									  a73 * k3 -> at(i) +
  // 									  a74 * k4 -> at(i) +
  // 									  a75 * k5 -> at(i) +
  // 									  a76 * k6 -> at(i) );
  // }
  // _compute_derivative();
  // _derivative_space -> copy_to_point(k7);

  // Compute Y_n = Y_{n-1} + h * ( b1 * k1 + ... + bs * ks)
  for (int i = 0; i < n; ++i)
  {
    yn->at(i) =
        start_integration_state->at(i) + _h * (b1 * k1->at(i) + b2 * k2->at(i) + b3 * k3->at(i) + b4 * k4->at(i) +
                                               b5 * k5->at(i) + b6 * k6->at(i) + b7 * k7->at(i));
  }
  // k7 -> multiply(h * b7);
  // k7 -> add_multiply(h * b6, k6);
  // k7 -> add_multiply(h * b5, k5);
  // k7 -> add_multiply(h * b4, k4);
  // k7 -> add_multiply(h * b3, k3);
  // k7 -> add_multiply(h * b2, k2);
  // k7 -> add_multiply(h * b1, k1);
  // _state_space -> copy_point(yn, start_integration_state);
  // yn -> add(k7);

  // Compute Y^h_n = Y_{n-1} + h * ( bh1 * k1 + ... + bhs * ks)
  // k_cache -> multiply(h * bh7);
  // k_cache -> add_multiply(h * bh6, k6);
  // k_cache -> add_multiply(h * bh5, k5);
  // k_cache -> add_multiply(h * bh4, k4);
  // k_cache -> add_multiply(h * bh3, k3);
  // k_cache -> add_multiply(h * bh2, k2);
  // k_cache -> add_multiply(h * bh1, k1);
  // _state_space -> copy_point(yhn, start_integration_state);
  // yhn -> add(k_cache);

  // Instead of computing Y^h_n, lets compute the difference
  // Y_n - Y^h_n = (Y_{n-1} + h * ( b1 * k1 + ... + bs * ks))
  // 			 	-(Y_{n-1} + h * ( bh1 * k1 + ... + bhs * ks))
  // 			   = 0 + h * b1 * k1 - h * bh1 * k1 + ... + h * bs * ks - h * bhs * ks
  // 			   = h * k1 (b1 - bh1) + ... + h * ks (bs - bhs)
  // 			   = h * (k1 (b1 - bh1) + ... + ks (bs - bhs))
  for (int i = 0; i < n; ++i)
  {
    k_cache->at(i) =
        _h * ((b1 - bh1) * k1->at(i) + (b2 - bh2) * k2->at(i) + (b3 - bh3) * k3->at(i) + (b4 - bh4) * k4->at(i) +
              (b5 - bh5) * k5->at(i) + (b6 - bh6) * k6->at(i) + (b7 - bh7) * k7->at(i));
  }

  // k_cache -> multiply( (b7 - bh7) );
  // k_cache -> add_multiply( (b6 - bh6), k6);
  // k_cache -> add_multiply( (b5 - bh5), k5);
  // k_cache -> add_multiply( (b4 - bh4), k4);
  // k_cache -> add_multiply( (b3 - bh3), k3);
  // k_cache -> add_multiply( (b2 - bh2), k2);
  // k_cache -> add_multiply( (b1 - bh1), k1);
  // k_cache -> multiply(h);
}

void dopri5_t::initial_h()
{
  double d0 = error(start_integration_state);
  double d1 = error(k1);
  double d2;
  double h0, h1;

  double der12;

  if (d1 <= 1.e-10 || d0 <= 1.e-10)
  {
    h0 = 1.0e-6;
  }
  else
  {
    h0 = (d0 / d1) * 0.01;
  }

  // PERFORM AN EXPLICIT EULER STEP
  _state_space->integrate(start_integration_state, _derivative_space, _h);
  _compute_derivative();
  _derivative_space->copy_to_point(k2);
  // ESTIMATE THE SECOND DERIVATIVE OF THE SOLUTION
  d2 = error(k1, k2) / h0;

  der12 = std::max(d2, d1);
  if (der12 < 1.e-15)
  {
    h1 = std::max(1.0e-6, h0 * 1.0e-3);
  }
  else
  {
    h1 = std::pow(0.01 / der12, 1.0 / 5.0);
  }
  _h = std::min(100 * h0, h1);

  _derivative_space->copy_from_point(k1);
}

void dopri5_t::integrate(const double simulation_step)
{
  using std::max;
  using std::min;

  double h_limit = h_initial;
  // if (simulation_step != 0.0) h_limit = simulation_step;
  double FAC11, FAC;
  bool rejected = false;
  const double FACC2 = 1.0 / facmax;
  const double FACC1 = 1.0 / facmin;
  const double EXPO1 = 0.2 - beta * 0.75;
  const double HMAX = simulation_step;
  double FACOLD = 1.0e-4;
  // double POSNEG=SIGN(1.D0,XEND-X);
  // double HLAMB=0.D0;
  // double IASTI=0
  double err, h_new;
  // _h = simulation_step <= 0.0 ? _h : simulation_step;
  _h = simulation_step;
  // h = h_initial;
  double h_accum = 0.0;
  // printf("-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~\n");
  do
  {
    if (h_accum + _h > h_limit)
      _h = h_limit - h_accum;
    // h = simulation_step;
    step();
    // k_cache =
    err = error(k_cache);
    // printf("err: %.6f\n", err );
    // err = 0.9;
    FAC11 = pow(err, EXPO1);
    FAC = FAC11 / pow(FACOLD, beta);
    FAC = max(FACC2, min(FACC1, FAC / safety));
    h_new = _h / FAC;
    // h = h * std::min(facmax, std::max(facmin, safety * pow( 1.0 / err, (1.0 / 5.0) )));
    // rejected = ;
    if (err > 1)
    {
      h_new = _h / min(FACC1, FAC11 / safety);
      _state_space->copy_from_point(start_integration_state);
    }
    else
    {
      _state_space->copy_from_point(yn);
      h_accum += _h;
      FACOLD = max(err, 1.0e-4);
    }
    // std::cout << "\terr: " << std::scientific;
    // 		std::cout << err ;
    // std::cout << "\th: " << std::scientific;
    // 		std::cout << h;
    // std::cout << "\th_accum: " << std::scientific;
    // 		std::cout << h_accum;
    // 		std::cout << "\th_limit: " << std::scientific;
    // 		std::cout << h_limit;
    // 		std::cout << std::endl;
    // std::cout << _state_space -> print_memory(4) << std::endl;

    _h = h_new;
  } while (h_accum < h_limit);
  // if(abs(h) > HMAX) h = HMAX;
  // h=min(HNEW,h_initial);
}
}  // namespace prx