#pragma once

// #include "playback/trajectory.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/streamer.hpp"

#include <deque>
#include <fstream>
#include <memory>

namespace prx
{
namespace experimental
{
/**
 * @authors Edgar Granados
 *
 */
template <typename Space, typename TimeType>
class trajectory_t
{
public:
  using SpacePtr = std::shared_ptr<Space>;
  using State = typename Space::State;
  using Time = TimeType;
  using StateTimePair = std::pair<State, Time>;
  // using Trajectory = trajectory_t<Space, TimeType>;

  using Iterator = typename std::deque<State>::iterator;
  using ConstIterator = typename std::deque<State>::const_iterator;

  trajectory_t() {};
  trajectory_t(const trajectory_t& traj) = default;

  ~trajectory_t() {};

  // static std::shared_ptr<Trajectory> create()
  // {
  //   return std::make_shared<Trajectory>();
  // }

  inline std::size_t size() const
  {
    return _states.size();
  }

  template <typename Idx, std::enable_if_t<std::is_integral_v<Idx>, bool> = true>
  inline StateTimePair operator[](const Idx index) const
  {
    return { _states[index], _stamps[index] };
    // return at(index);
  }

  inline State front() const
  {
    return _states.front();
  }

  inline State back() const
  {
    return _states.back();
  }

  inline Iterator begin()
  {
    return _states.begin();
  }

  inline Iterator end()
  {
    return _states.end();
  }

  inline ConstIterator begin() const
  {
    return _states.begin();
  }

  inline ConstIterator end() const
  {
    return _states.end();
  }

  // Return the state at ti.
  State at(const Time ti) const
  {
    auto it = std::lower_bound(_stamps.begin(), _stamps.end(), ti);
    prx_assert(it != _stamps.end(), "Trying to access a trajectory outside of bounds");
    auto idx = std::distance(_stamps.begin(), it);
    if (idx == _states.size() - 1)
    {
      return _states[idx];
    }
    return _state_space->interpolator(_states[idx], _states[idx + 1]);
  }

  // Split at specified time: ThisOld = [ ThisNew | NewTraj ]
  // trajectory_t split(const double time_of_split){}

  double duration() const
  {
    return _stamps.back() - _stamps.front();
  }

  void push_back(const State xi, const Time ti)
  {
    _states.push_back(xi);
    _stamps.push_back(ti);
  }

  // void push_back(const trajectory_t& other)
  // {
  //   for (int i = 0; i < other.size(); ++i)
  //   {
  //     auto pair = other[i];
  //     _states.push_back(pair.first);
  //     _stamps.push_back(pair.second);
  //   }
  // }

  void clear()
  {
    _states.clear();
    _stamps.clear();
  }

  void to_file(const std::string filename, const std::ios_base::openmode mode = std::ofstream::trunc) const
  {
    std::ofstream ofs(filename, mode);
    ofs << *this;
    ofs.close();
  }

  friend std::ostream& operator<<(std::ostream& os, const trajectory_t& obj)
  {
    for (int i = 0; i < obj.size(); ++i)
    {
      prx::streamer_t<State>::to_stream(os, obj._states[i]);
      prx::streamer_t<TimeType>::to_stream(os, obj._stamps[i]);
      os << "\n";
    }
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<trajectory_t>& obj)
  {
    os << *obj;
  }

protected:
  SpacePtr _state_space;

  std::deque<State> _states;
  std::deque<Time> _stamps;
};
}  // namespace experimental

template <typename Space, typename TimeType>
void merge(experimental::trajectory_t<Space, TimeType>& traj, const experimental::trajectory_t<Space, TimeType> other)
{
  TimeType last{ traj[traj.size() - 1].second };
  for (int i = 0; i < other.size(); ++i)
  {
    auto pair = other[i];
    last += pair.second;
    traj.push_back(pair.first, last);
  }
}
// template <typename Space>
// void merge(experimental::trajectory_t<Space, OTHERTYPE> traj, const experimental::trajectory_t<Space, OTHERTYPE>
// other)
// {

//   for (int i = 0; i < other.size(); ++i)
//   {
//     auto pair = other[i];
//     traj.push_back(pair.first, pair.second);
//   }
// }

}  // namespace prx
