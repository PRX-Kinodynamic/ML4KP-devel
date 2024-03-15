#pragma once

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/defs.hpp"

#include <deque>

namespace prx
{
extern double simulation_step;
/**
 * @brief <b>A class that defines a trajectory.</b>
 *
 * A trajectory consists of a sequence of states.
 *
 * @authors Zakary Littlefield
 *
 */
class trajectory_t
{
public:
  typedef std::vector<space_point_t>::iterator iterator;
  typedef std::vector<space_point_t>::const_iterator const_iterator;

  trajectory_t(const space_t* space);
  trajectory_t(const trajectory_t& traj);

  template <typename Container>
  trajectory_t(const space_t* space, const Container container) : trajectory_t(space)
  {
    for (auto state : container)
    {
      copy_onto_back(state);
    }
  }

  ~trajectory_t();

  inline unsigned size() const
  {
    return num_states;
  }
  inline space_point_t operator[](unsigned index) const
  {
    return at(index);
  }
  inline space_point_t operator[](double index) const
  {
    return at(index);
  }

  inline space_point_t front() const
  {
    prx_assert(num_states != 0, "Trying to access the front of a trajectory with zero size.");
    return states[0];
  }

  inline space_point_t back() const
  {
    prx_assert(num_states != 0, "Trying to access the back of a trajectory with zero size.");
    return states[num_states - 1];
  }

  inline iterator begin()
  {
    return states.begin();
  }

  inline iterator end()
  {
    return end_iterator;
  }

  inline const_iterator begin() const
  {
    return states.begin();
  }

  inline const_iterator end() const
  {
    return const_end_iterator;
  }

  space_point_t at(unsigned index) const
  {
    prx_assert(index < num_states, "Trying to access state outside of trajectory size.");
    return states[index];
  }
  // Return the state at t. If normalized_input is true, then t \in [0,1]. Otherwise, t \in [0, duration]
  space_point_t at(const double t, const bool normalized_input = true) const
  {
    double t01{ t };
    if (not normalized_input)
    {
      t01 = t / duration();  // current t \in [0,1]
    }
    prx_assert(t01 <= 1.0, "Requested trajectory state at [" << t << "] out of range.");
    return interpolate(t01);
  }

  std::size_t index_at_time(const double ti) const;

  unsigned get_num_states() const
  {
    return num_states;
  }

  double duration() const
  {
    return (size() - 1) * simulation_step;
  }

  void resize(unsigned num_size);

  void pop_back();

  void copy(const trajectory_t& t);
  trajectory_t& operator=(const trajectory_t& t);
  trajectory_t& operator+=(const trajectory_t& t);
  bool operator==(const trajectory_t& t);
  bool operator!=(const trajectory_t& t);

  void clear();
  // void copy_onto_back(space_point_t state);
  void copy_onto_back(const space_t* space);

  template <typename State>
  void copy_onto_back(const State state)
  {
    if ((num_states + 1) >= max_num_states)
    {
      increase_buffer();

      end_iterator = states.begin();
      const_end_iterator = states.begin();
      std::advance(end_iterator, num_states);
      std::advance(const_end_iterator, num_states);
    }
    state_space->copy(*end_iterator, state);
    ++end_iterator;
    ++const_end_iterator;
    ++num_states;
  }

  std::string print(unsigned precision = 3) const;

  friend std::ostream& operator<<(std::ostream& os, const trajectory_t& obj)
  {
    os << obj.print() << " ";
    // os << std::endl;
    return os;
  }

protected:
  space_point_t interpolate(double s) const;

  void increase_buffer();
  const space_t* state_space;

  iterator end_iterator;
  const_iterator const_end_iterator;
  unsigned max_num_states;
  unsigned num_states;
  std::vector<space_point_t> states;
};
}  // namespace prx
