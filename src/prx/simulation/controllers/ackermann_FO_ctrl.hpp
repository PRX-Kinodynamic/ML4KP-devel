
#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/ackermann_FO.hpp"

namespace prx
{

class ackermann_FO_ctrl_t : public controller_t
{
public:
  ackermann_FO_ctrl_t(const ackermann_FO_ctrl_t& other) : controller_t(other)
  {
    direccion = 0;
  }

  ackermann_FO_ctrl_t(system_ptr_t _plant, std::string _name = "ackermann_FO_controller") : controller_t(_plant, _name)
  {
    direccion = 0;
    aFO = std::dynamic_pointer_cast<ackermann_FO>(_plant);
    aux_pt = aFO->get_state_space()->make_point();
    ctrl_pt = aFO->get_control_space()->make_point();
    goal = aFO->get_state_space()->make_point();
  }
  virtual ~ackermann_FO_ctrl_t(){};

  virtual void compute_controls() override;

  void set_gains(double _k_rho, double _k_alpha, double _k_beta);

private:
  void movepoint_sfunc(double x, double y, double theta);

  std::shared_ptr<ackermann_FO> aFO;
  space_point_t aux_pt;
  space_point_t ctrl_pt;

  double rho;
  double alpha;
  double beta;

  double k_rho = 1;
  double k_alpha = 0.2;
  double k_beta = 0.1;

  double direccion;

  double gamma;
  double V;

  // double x, y, theta;
};

}  // namespace prx
