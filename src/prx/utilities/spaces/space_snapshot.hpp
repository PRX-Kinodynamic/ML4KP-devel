#pragma once

#include <vector>
#include <string>
#include <memory>
#include <numeric>

#include "prx/utilities/defs.hpp"

namespace prx
{
class space_t;           // Fwd declaration
class space_snapshot_t;  // Fwd declaration
typedef std::shared_ptr<space_snapshot_t> space_point_t;

/**
 * @brief <b> Stores a single point in a space. </b>
 *
 * Stores a single point in a space.
 *
 * @author Zakary Littlefield
 */
class space_snapshot_t
{
public:
  typedef std::vector<double>::iterator iterator;
  typedef std::vector<double>::const_iterator const_iterator;

  /**
   * @brief Destructor.
   *
   * Clears the memory of the space snapshot.
   */
  ~space_snapshot_t()
  {
    _memory.clear();
  }

  /**
   * @brief Gets the value of the space snapshot along the specified dimension.
   * @param index The index of the dimension.
   * @return Value of the space snapshot at the index.
   */
  double& at(std::size_t index)
  {
    return _memory[index];
  }

  /**
   * @brief Gets the value of the space snapshot along the specified dimension.
   * @param index The index of the dimension.
   * @return Value of the space snapshot at the index.
   */
  const double at(std::size_t index) const
  {
    return _memory[index];
  }

  double& operator[](const std::size_t idx)
  {
    return _memory[idx];
  }

  inline const double operator[](const std::size_t idx) const
  {
    return _memory[idx];
  }

  /**
   * @brief Gets the dimensionality of the space snapshot.
   * @return Dimensionality of the space snapshot.
   */
  inline const std::size_t size() const
  {
    return _memory.size();
  }

  /**
   * @brief Gets the dimensionality of the space snapshot.
   * @return Dimensionality of the space snapshot.
   */
  inline const unsigned int get_dim() const
  {
    return size();
  }

  void add(const space_point_t& obj)
  {
    prx_assert(obj->get_dim() == _memory.size(),
               "Points have different dimension size!: " << obj->get_dim() << " and " << _memory.size());
    Vec(*this) += Vec(obj);
  }

  void multiply(const double& sclr)
  {
    Vec(*this) = sclr * Vec(*this);
  }

  /**
   * @brief Performs yn <- yn + sclr * pt
   * @details Performs yn <- yn + sclr * pt. Same as add if sclr = 1. Points don't need to be in the same space.
   *
   * @param sclr Scalar
   * @param pt Point to add.
   */
  void add_multiply(const double& sclr, const space_point_t& pt)
  {
    prx_assert(pt->get_dim() == _memory.size(),
               "Points have different dimension size!: " << pt->get_dim() << " and " << _memory.size());
    Vec(*this) = Vec(*this) + sclr * Vec(pt);
  }

  iterator begin()
  {
    return _memory.begin();
  }
  iterator end()
  {
    return _memory.end();
  }

  const_iterator begin() const
  {
    return _memory.begin();
  }
  const_iterator end() const
  {
    return _memory.end();
  }

  friend std::ostream& operator<<(std::ostream& os, const space_point_t& obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const space_snapshot_t& obj)
  {
    os << std::fixed << std::setprecision(prx::constants::precision);
    for (auto e : obj._memory)
    {
      os << e << prx::constants::separating_value;
    }
    return os;
  }

  operator std::string() const
  {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  /**
   * @brief      Returns a new object of type Vector with a copy of data in memory
   *
   * @tparam     Vector  Type of the returned value
   *
   * @return     Copy of the data in this object
   */
  template <typename Vector = Eigen::VectorXd>
  inline Vector as()
  {
    return Vector{ _map_vector };
  }

  /**
   * @brief      Returns a mutable view (Map) to the values as Eigen::Vector. No copy is performed. No bounds check.
   *
   * @param[in]  space_point   The space_point to use
   *
   * @return     A mutable Eigen::Map.
   */
  inline friend Eigen::Map<Eigen::VectorXd>& Vec(space_point_t space_point)
  {
    return space_point->_map_vector;
  }

  /**
   * @brief      Returns a mutable view (Map) to the values as Eigen::Vector. No copy is performed. No bounds check.
   *
   * @param[in]  space_snapshot   The space_snapshot to use
   *
   * @return     A mutable Eigen::Map.
   */
  inline friend Eigen::Map<Eigen::VectorXd>& Vec(space_snapshot_t& space_snapshot)
  {
    return space_snapshot._map_vector;
  }

protected:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  in_parent  Parent space
   */
  space_snapshot_t(const space_t* const in_parent, const std::size_t dim);

  const space_t* const _parent;
  std::vector<double> _memory;
  Eigen::Map<Eigen::VectorXd> _map_vector;

  friend class space_t;
};
}  // namespace prx