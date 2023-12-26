#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"

#include <memory>

// [1] Karaman, Sertac, and Emilio Frazzoli. "Sampling-based algorithms for optimal motion planning." The international
//     journal of robotics research 30, no. 7 (2011): 846-894.
namespace prx
{

extern double simulation_step;
class controller_t;
class system_controller_t;

class system_t;
// class system_ptr_t;
typedef std::shared_ptr<system_t> system_ptr_t;

template <class T>
system_ptr_t create_system(const std::string& path)
{
  system_ptr_t new_ptr;
  new_ptr.reset(new T(path));
  return new_ptr;
}

bool is_child_system(system_ptr_t parent, system_ptr_t child);

class system_t : public std::enable_shared_from_this<system_t>
{
public:
  system_t(const std::string& path);
  virtual ~system_t();

  virtual inline space_t* get_state_space() const
  {
    return state_space;
  }
  inline space_t* get_control_space() const
  {
    return input_control_space;
  }
  inline space_t* get_parameter_space() const
  {
    return parameter_space;
  }

  virtual void add_system(system_ptr_t&) = 0;

  // Keeping this for analytical plants.
  virtual void propagate(const double simulation_step) = 0;

  virtual void compute_control() = 0;

  virtual void compute_stopping_maneuver(space_point_t, std::vector<double>&, std::vector<double>&);
  virtual void finalize_system_tree()
  {
    PRX_NOT_IMPLEMENTED
    // default do nothing because you don't have any subsystems
  }

  virtual void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) = 0;

  // virtual bool linearize(space_point_t xt, space_point_t ut, double epsilon = 1e-3)

  virtual bool linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D,
                         space_point_t xt = nullptr, space_point_t ut = nullptr, double epsilon = 1e-3)
  {
    PRX_NOT_IMPLEMENTED
    return false;
  };

  // Steering function[1]:
  // Given two points x,y ∈ X:
  //      Steer:  (x,y) -> z
  // returns a point z ∈ X such that z is “closer” to y than x is.
  // This version performs the copies from/to the given points
  void steer(space_point_t z, const space_point_t x, const space_point_t y, const double ti)
  {
    this->steer(x, y, ti);
    state_space->copy_to(z);
  }

  // Same as steer(x,y)->z. The resulting z will be in memory.
  // This function should be overwritten by any plant that has a steering function available
  virtual void steer(const space_point_t x, const space_point_t y, const double ti)
  {
    PRX_NOT_IMPLEMENTED
  }

  inline std::string get_pathname()
  {
    return pathname;
  }

  inline plant_type get_system_type()
  {
    return system_type;
  }

  friend std::ostream& operator<<(std::ostream& os, const system_t& obj)
  {
    os << "state_space: " << *obj.state_space << "\tcontrol_space: " << *obj.input_control_space;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const system_ptr_t& obj)
  {
    os << "state_space: " << *(obj->state_space) << "\tcontrol_space: " << *(obj->input_control_space);
    return os;
  }

protected:
  space_t* state_space;
  space_t* input_control_space;
  space_t* parameter_space;

  system_t(const system_ptr_t other)
  {
    state_space = other->state_space;
    input_control_space = other->input_control_space;
    parameter_space = other->parameter_space;
    parent_system = other->parent_system;
    pathname = other->pathname;
    system_type = other->system_type;
    owned_values = false;
    // state_memory = other -> state_memory;
    // control_memory = other -> control_memory;
    // parameter_memory = other -> parameter_memory;
  }

  std::weak_ptr<system_t> parent_system;

  void set_parent_system(system_ptr_t parent)
  {
    if (!parent_system.expired())
      prx_throw("Trying to add system " << pathname << " underneath " << parent->get_pathname() << " when "
                                        << parent_system.lock()->get_pathname() << " is already its parent.");
    parent_system = parent;
  }

  std::string pathname;
  plant_type system_type;

  std::vector<double*> state_memory;
  std::vector<double*> control_memory;
  std::vector<double*> parameter_memory;

  bool owned_values;

private:
  system_t()
  {
  }
  friend controller_t;
  friend system_controller_t;
};
}  // namespace prx
