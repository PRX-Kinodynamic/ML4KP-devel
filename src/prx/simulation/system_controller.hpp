#pragma once

#include <memory>
#include <unordered_map>
// #include "prx/simulation/system.hpp"
#include "prx/simulation/plant.hpp"

namespace prx
{

class system_controller_t : public system_t
{
public:
  system_controller_t(system_ptr_t plant, const std::string& path)
    : system_t(path), _plant(std::dynamic_pointer_cast<prx::plant_t>(plant))
  {
    subsystems.clear();
    composite_state_space = nullptr;
  }
  virtual ~system_controller_t();

  virtual inline space_t* get_state_space() const override final
  {
    return composite_state_space;
  }

  virtual void add_system(system_ptr_t&) override final;

  virtual void compute_control() override;

  virtual void propagate(const double simulation_step) override;

  virtual void finalize_system_tree() override;

  virtual void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override{};

protected:
  std::shared_ptr<plant_t> _plant;
  space_t* composite_state_space;

  std::unordered_map<std::string, system_ptr_t> subsystems;
};
}  // namespace prx
