#pragma once

#include "prx/utilities/defs.hpp"

#include <vector>
#include <string>
#include <memory>
#include <numeric>

namespace prx
{
class space_t;
class space_snapshot_t;
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
    memory.clear();
  }

  /**
   * @brief Gets the value of the space snapshot along the specified dimension.
   * @param index The index of the dimension.
   * @return Value of the space snapshot at the index.
   */
  double& at(unsigned index)
  {
    return memory[index];
  }

  /**
   * @brief Gets the value of the space snapshot along the specified dimension.
   * @param index The index of the dimension.
   * @return Value of the space snapshot at the index.
   */
  const double at(unsigned index) const
  {
    return memory[index];
  }

  space_snapshot_t(const space_t* const in_parent);

  /**
   * @brief Gets the dimensionality of the space snapshot.
   * @return Dimensionality of the space snapshot.
   */
  inline const std::size_t size()
  {
    return memory.size();
  }

  /**
   * @brief Gets the dimensionality of the space snapshot.
   * @return Dimensionality of the space snapshot.
   */
  inline const unsigned int get_dim()
  {
    return memory.size();
  }

  void add(const space_point_t& obj)
  {
    prx_assert(obj->get_dim() == memory.size(),
               "Points have different dimension size!: " << obj->get_dim() << " and " << memory.size());

    for (int i = 0; i < memory.size(); ++i)
    {
      memory[i] += obj->at(i);
    }
    // return shared_from_this();
  }

  void multiply(const double& sclr)
  {
    for (int i = 0; i < memory.size(); ++i)
    {
      memory[i] *= sclr;
    }
    // return shared_from_this();
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
    prx_assert(pt->get_dim() == memory.size(),
               "Points have different dimension size!: " << pt->get_dim() << " and " << memory.size());
    for (int i = 0; i < memory.size(); ++i)
    {
      memory[i] += (sclr * pt->at(i));
    }
  }

  iterator begin()
  {
    return memory.begin();
  }
  iterator end()
  {
    return memory.end();
  }

  const_iterator begin() const
  {
    return memory.begin();
  }
  const_iterator end() const
  {
    return memory.end();
  }

  double& operator[](const int i)
  {
    return at(i);
  }

  friend std::ostream& operator<<(std::ostream& os, const space_point_t& obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const space_snapshot_t& obj)
  {
    for (auto e : obj.memory)
    {
      os << e << " ";
    }
    // os << std::endl;
    return os;
  }

protected:
  const space_t* const parent;

  std::vector<double> memory;

  friend class space_t;

private:
  space_snapshot_t() : parent(nullptr)
  {
    memory.clear();
  }
};

typedef std::function<double(const space_point_t&, const space_point_t&)> distance_function_t;

/**
 * @brief <b> Defines a space.</b>
 *
 * A space is a set of points with some added structure. This class is used to
 * represent the state and control spaces of a plant.
 *
 * @author Zakary Littlefield.
 */
class space_t
{
public:
  enum class topology_t
  {
    EUCLIDEAN = 0,
    ROTATIONAL = 1,
    DISCRETE = 2,
    QUATERNION = 3,
    IDLE = 4
  };

  space_t(const std::string& topology, const std::vector<double*>& addresses, const std::string& name);

  space_t(const std::string& topology, const std::vector<double*>& addresses);

  space_t(const std::vector<const space_t*>& spaces);

  ~space_t();

  void set_bounds(const std::vector<double>& lower, const std::vector<double>& upper);

  space_point_t make_point() const;
  space_point_t clone_point(const space_point_t& point) const;

  /**
   * @brief The union of two space point
   * @details The union of two space points: res.dim == p1.dim + p2.dim, where res.dim is the dimension of
   * this space. The concatenation of the name spaces must match the following:
   * p1.name_space|p2.name_space == this.name_space
   *
   *
   * @param p1 The first point
   * @param p2 The second point
   *
   * @return The new point
   */
  void point_union(const space_point_t& p1, const space_point_t& p2, const space_point_t& pu);

  /**
   * @brief Splits a point into two points according to its space
   * @details Splits a point of Space C = A U B to point p1 \in A and p2 \in B
   *
   * @param ps_to_split Point to be split
   * @param ps1 Output point in the first space
   * @param ps2 Output point in the second space
   */
  void split_point(const space_point_t& ps_to_split, const space_point_t& ps1, const space_point_t& ps2);

  /**
   * @brief Check whether the given point is in this space
   * @details Check whether the given point is in this space
   *
   * @param pt The point to check
   * @return True if the point belongs to this space, false otherwise
   */
  bool is_point_in_space(const space_point_t& pt)
  {
    return space_name == pt->parent->space_name;
  }

  /**
   * @brief Add two points (vector addition)
   * @details Element-wise addition of two points belonging to the same space.
   *
   * @param point1 First point
   * @param point2 Second point
   * @param point3 Resulting point
   *
   */
  // void point_addition(const space_point_t& pt1, const space_point_t& pt2, const space_point_t& pt_res);

  bool equal_points(const space_point_t& point1, const space_point_t& point2) const;

  /**
   * @brief      Copy current state to a Eigen::VectorXd.
   *
   * @param[in]  Vector to copy the current space memory to.
   */
  virtual void copy_to_vector(Eigen::VectorXd& _v) const;

  /**
   * @brief      Copy from a Eigen::VectorXd.
   *
   * @param[in]  Vector to copy from.
   */
  virtual void copy_from_vector(const Eigen::VectorXd& _v) const;

  /**
   * @brief      Copy from a std::vector<double>
   *
   * @param[in]  Vector to copy from.
   */
  // void copy_from_vector(const std::vector<double>& source) const;

  /**
   * @brief      Copy current memory of the space to the given space point
   *
   * @param[in/out]  point  The point to copy to.
   */
  virtual void copy_to_point(const space_point_t& point) const;

  virtual void copy_from_point(const space_point_t& point) const;
  virtual void copy_point(const space_point_t& destination, const space_point_t& source) const;

  /**
   * @brief      Copy current memory of the space to the given vector
   *
   * @param      destination  The std::vector to copy to
   */
  virtual void copy_to_vector(std::vector<double>& destination) const;
  virtual void copy_from_vector(const std::vector<double>& source);

  void copy_point_from_vector(const space_point_t& destination, const std::vector<double>& source) const;
  void copy_point_from_vector(const space_point_t& destination, Eigen::Ref<Eigen::VectorXd> source) const;
  void copy_vector_from_point(std::vector<double>& destination, const space_point_t& source) const;
  void copy_vector_from_point(Eigen::Ref<Eigen::VectorXd> destination, const space_point_t& source) const;

  template <typename T, std::enable_if_t<(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true,
            typename F, std::enable_if_t<(std::is_pointer<F>{} || prx::utils::is_shared_ptr<F>{}), bool> = true>
  void copy(T& to, const F& from) const
  {
    prx_assert(from->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                      << typeid(decltype(*from)).name() << " ( "
                                                                      << from->size() << " )");
    prx_assert(to->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                    << typeid(decltype(*to)).name() << " ( "
                                                                    << to->size() << " )");
    for (int i = 0; i < from->size(); ++i)
    {
      to->at(i) = from->at(i);
    }
  }

  template <typename T, std::enable_if_t<(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true,
            typename F, std::enable_if_t<!(std::is_pointer<F>{} || prx::utils::is_shared_ptr<F>{}), bool> = true>
  void copy(T& to, const F& from) const
  {
    prx_assert(from.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                     << typeid(decltype(from)).name() << " ( "
                                                                     << from.size() << " )");
    prx_assert(to->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                    << typeid(decltype(*to)).name() << " ( "
                                                                    << to->size() << " )");
    for (int i = 0; i < from.size(); ++i)
    {
      to->at(i) = from.at(i);
    }
  }

  template <typename T, std::enable_if_t<!(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true,
            typename F, std::enable_if_t<(std::is_pointer<F>{} || prx::utils::is_shared_ptr<F>{}), bool> = true>
  void copy(T& to, const F& from) const
  {
    prx_assert(from->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                      << typeid(decltype(*from)).name() << " ( "
                                                                      << from->size() << " )");
    prx_assert(to.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                   << typeid(decltype(to)).name() << " ( " << to.size()
                                                                   << " )");
    for (int i = 0; i < from->size(); ++i)
    {
      to.at(i) = from->at(i);
    }
  }

  template <typename T, std::enable_if_t<!(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true,
            typename F, std::enable_if_t<!(std::is_pointer<F>{} || prx::utils::is_shared_ptr<F>{}), bool> = true>
  void copy(T& to, const F& from) const
  {
    prx_assert(from.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                     << typeid(decltype(from)).name() << " ( "
                                                                     << from.size() << " )");
    prx_assert(to.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                   << typeid(decltype(to)).name() << " ( " << to.size()
                                                                   << " )");
    for (int i = 0; i < from.size(); ++i)
    {
      to.at(i) = from.at(i);
    }
  }

  // Explicit std::initializer_list copy is needed due to the § 14.8.2.5/5 C++11 standard
  // (std::initializer_list is a non-deduced context for a template argument)
  // No need for [T=prx::space_point_t] since we know from is initializer_list
  template <typename T, std::enable_if_t<(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true,
            typename F>
  inline void copy(T& to, const std::initializer_list<F> from) const
  {
    prx_assert(from.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                     << typeid(decltype(from)).name() << " ( "
                                                                     << from.size() << " )");
    std::vector<F> aux_vector = from;
    copy(to, aux_vector);
  }

  template <typename T, std::enable_if_t<(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true>
  void copy_from(const T& from)
  {
    prx_assert(from->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                      << typeid(decltype(*from)).name() << " ( "
                                                                      << from->size() << " )");
    if (std::static_pointer_cast<space_snapshot_t>(from) != nullptr)
    {
      const space_point_t aux = std::static_pointer_cast<space_snapshot_t>(from);
      prx_assert(aux->parent->space_name == space_name,
                 "Point is of space ( " << aux->parent->space_name << " ), but space is ( " << space_name << ").");
    }

    for (int i = 0; i < from->size(); ++i)
    {
      *addresses[i] = from->at(i);
    }
  }

  template <typename T, std::enable_if_t<!std::is_pointer<T>{} && !prx::utils::is_shared_ptr<T>{}, bool> = true>
  void copy_from(const T& from)
  {
    prx_assert(from->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                      << typeid(decltype(from)).name() << " ( "
                                                                      << from.size() << " )");
    for (int i = 0; i < from.size(); ++i)
    {
      *addresses[i] = from.at(i);
    }
  }

  // template <typename T, typename std::enable_if_t<std::is_pointer<T>::value || prx::utils::is_shared_ptr<T>::value>>
  template <typename T, std::enable_if_t<(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true>
  void copy_to(T& to) const
  {
    prx_assert(to->size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                    << typeid(decltype(*to)).name() << " ( "
                                                                    << to->size() << " )");
    if (std::static_pointer_cast<space_snapshot_t>(to) != nullptr)
    {
      const space_point_t aux = std::static_pointer_cast<space_snapshot_t>(to);
      prx_assert(aux->parent->space_name == space_name,
                 "Point is of space ( " << aux->parent->space_name << " ), but space is ( " << space_name << ").");
    }

    for (int i = 0; i < to->size(); ++i)
    {
      to->at(i) = *addresses[i];
    }
  }

  template <typename T, std::enable_if_t<!(std::is_pointer<T>{} || prx::utils::is_shared_ptr<T>{}), bool> = true>
  void copy_to(T& to) const
  {
    prx_assert(to.size() == dimension, "Mismatch on point sizes. " << space_name << " ( " << dimension << " ) vs "
                                                                   << typeid(decltype(to)).name() << " ( " << to.size()
                                                                   << " )");

    for (int i = 0; i < to.size(); ++i)
    {
      to.at(i) = *addresses[i];
    }
  }

  const inline std::size_t size() const
  {
    return dimension;
  }

  const inline unsigned int get_dimension() const
  {
    return dimension;
  }

  void enforce_bounds(const space_point_t& point) const;
  void enforce_bounds() const;
  bool satisfies_bounds(const space_point_t& point) const;
  virtual void sample(const space_point_t& point) const;

  inline std::string get_space_name() const
  {
    return space_name;
  }

  inline double& at(unsigned index) const
  {
    prx_assert(index < dimension, "Trying index into space at index " << index << " with dimension " << dimension);
    return *addresses[index];
  }

  inline double& operator[](unsigned index) const
  {
    return at(index);
  }

  inline double get_lower_bound(unsigned i) const
  {
    prx_assert(i < dimension,
               "Error: Trying to get bound for " << i << " that is higher than state dimension " << dimension << ".");
    return *lower_bounds[i];
  }

  inline double get_upper_bound(unsigned i) const
  {
    prx_assert(i < dimension,
               "Error: Trying to get bound for " << i << " that is higher than state dimension " << dimension << ".");
    return *upper_bounds[i];
  }

  std::vector<std::pair<double, double>> get_bounds() const;

  std::vector<double> get_upper_bounds() const;

  std::vector<double> get_lower_bounds() const;

  void print_bounds() const;

  void integrate(const space_point_t& point, const space_t* derivative, double delta_t);
  void integrate(const space_t* derivative, double delta_t);

  void interpolate(const space_point_t& point1, const space_point_t& point2, double t, space_point_t& result) const;

  std::string print_point(const space_point_t& point, unsigned prec = 25) const;

  std::string print_memory(unsigned prec = 25) const;

  friend std::ostream& operator<<(std::ostream& os, const space_t& obj)
  {
    os << obj.print_memory(3);
    return os;
  }

  static double l1_norm(const space_point_t& p1)
  {
    auto fn = [&](double accum, double e) { return accum + std::abs(e); };

    return std::accumulate(p1->begin(), p1->end(), 0.0, fn);
  }

  static double l1_norm(const space_point_t& p1, const space_point_t& p2)
  {
    // int i = 0;
    auto fn = [](double accum, std::tuple<space_snapshot_t::iterator, space_snapshot_t::iterator>& e) {
      double e1, e2;
      std::tie(e1, e2) = unzip(e);

      return accum + std::abs(e1 - e2);
    };

    auto zipped = zip_iters(p1, p2);

    return std::accumulate(zipped.begin(), zipped.end(), 0.0, fn);
  }

  static double l2_norm(const space_point_t& p1)
  {
    auto fn = [&](double accum, double e) { return accum + std::pow(e, 2.0); };

    return std::sqrt(std::accumulate(p1->begin(), p1->end(), 0.0, fn));
  }

  static double l2_norm(const space_point_t& p1, const space_point_t& p2)
  {
    // int i = 0;
    auto fn = [](double accum, std::tuple<space_snapshot_t::iterator, space_snapshot_t::iterator>& e) {
      double e1, e2;
      std::tie(e1, e2) = unzip(e);

      return accum + std::pow(e1 - e2, 2.0);
    };
    auto zipped = zip_iters(p1, p2);

    return std::sqrt(std::accumulate(zipped.begin(), zipped.end(), 0.0, fn));
  }

  static double lp_norm(const space_point_t& p1, const double p)
  {
    auto fn = [&](double accum, double e) { return accum + std::pow(e, p); };

    return std::pow(std::accumulate(p1->begin(), p1->end(), 0.0, fn), 1.0 / p);
  }

  static double lp_norm(const space_point_t& p1, const space_point_t& p2, const double p)
  {
    double e1, e2;
    double accum = 0;
    for (auto e : zip_iters(p1, p2))
    {
      std::tie(e1, e2) = unzip(e);
      accum += std::pow(e1 - e2, p);
    }
    return std::pow(accum, 1.0 / p);
  }

  /**
   * @brief      Compute the euclidean distance between two points using dimensions
   *             [i_{begin}, i_{end}). \sqrt{ (p1[i_{begin}] - p2[i_{begin}])^2 }
   *
   *
   * @param[in]  p1     First point
   * @param[in]  p2     Second point
   * @param[in]  i_begin  Start dimention, default is 0.
   * @param[in]  i_end    Stopping dimension, default is 2.
   *
   * @return     { description_of_the_return_value }
   */
  static double euclidean_2d(const space_point_t& p1, const space_point_t& p2, int start = 0, int end = 2)
  {
    double e1, e2;
    double accum = 0;
    int i = start;
    for (auto e : zip_iters(p1, p2))
    {
      if (i < start)
        continue;
      if (i >= end)
        break;
      std::tie(e1, e2) = unzip(e);
      accum += std::pow(e1 - e2, 2.0);
      i += 1;
    }
    return std::sqrt(accum);
  }

  static double euclidean_distance(const space_t* s, const space_point_t& p2)
  {
    double accum = 0;
    for (int i = 0; i < s->get_dimension(); ++i)
    {
      accum += std::pow((*p2)[i] - s->at(i), 2);
    }

    return std::sqrt(accum);
  }

protected:
  space_t(const space_t* other)
  {
    dimension = other->dimension;
    for (int i = 0; i < dimension; ++i)
    {
      addresses.push_back(other->addresses[i]);
      lower_bounds.push_back(other->lower_bounds[i]);
      upper_bounds.push_back(other->upper_bounds[i]);
      topology.push_back(other->topology[i]);
    }

    // addresses = other -> addresses;
    // lower_bounds = other -> lower_bounds;
    // upper_bounds = other -> upper_bounds;
    space_name = other->space_name;
    // topology = other -> topology;
    owned_values = false;
  }

  std::size_t dimension;
  std::vector<double*> addresses;
  std::vector<double*> lower_bounds;
  std::vector<double*> upper_bounds;
  std::vector<topology_t> topology;
  std::string space_name;
  bool owned_values;

  space_t(){};
};

typedef std::shared_ptr<space_t> space_ptr_t;
}  // namespace prx