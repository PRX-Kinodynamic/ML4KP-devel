#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/utilities/spaces/noisy_space.hpp"

#include <memory>

namespace prx
{
template <class T>
class noisy_plant_t : public plant_t
{
public:
  noisy_plant_t(const std::string& path) = delete;

  template <class... Types>
  noisy_plant_t(const system_ptr_t& _sys_ptr, Types... args) : plant_t(_sys_ptr)
  {
    auto _plant = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
    prx_assert(_plant != nullptr, "Problem casting to a plant_t");
    plant = _plant;
    // noisy_space = new noisy_space_t<T>(_sys_ptr -> get_state_space(), args...);
    noisy_space = std::make_shared<noisy_space_t<T>>(_sys_ptr->get_state_space(), args...);
  }

  virtual ~noisy_plant_t()
  {
  }

  virtual inline space_t* get_state_space() const override
  {
    return noisy_space.get();
    // return static_cast<space_t*>(noisy_space);
  }

  virtual void update_configuration() override
  {
    plant->update_configuration();
  }

  virtual void compute_derivative() override
  {
    plant->compute_derivative();
  }

  virtual bool linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D,
                         space_point_t xt = nullptr, space_point_t ut = nullptr) override
  {
    if (xt != nullptr)
    {
      get_state_space()->copy_from(xt);
      get_control_space()->copy_from(ut);
    }
    return plant->linearize(A, B);
  }

private:
  std::shared_ptr<plant_t> plant;
  std::shared_ptr<noisy_space_t<T>> noisy_space;
};
}  // namespace prx