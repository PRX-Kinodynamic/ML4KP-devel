#pragma once

#include <vector>
#include <string>
#include <memory>
#include <numeric>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/template_utils.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/utilities/spaces/space_snapshot.hpp"

namespace prx
{
class space_t;
typedef std::shared_ptr<space_snapshot_t> space_point_t;

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
  inline bool is_point_in_space(const space_point_t& pt) const
  {
    return space_name == pt->_parent->space_name;
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
  virtual void copy_from_vector(const Eigen::VectorXd& _v);

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
  virtual void copy_to_point(const space_point_t point) const;

  virtual void copy_from_point(const space_point_t point) const;
  virtual void copy_point(const space_point_t destination, const space_point_t source) const;

  /**
   * @brief      Copy current memory of the space to the given vector
   *
   * @param      destination  The std::vector to copy to
   */
  virtual void copy_to_vector(std::vector<double>& destination) const;
  virtual void copy_from_vector(const std::vector<double>& source);

  void copy_point_from_vector(space_point_t destination, const std::vector<double>& source) const;
  void copy_point_from_vector(space_point_t destination, Eigen::Ref<Eigen::VectorXd> source) const;
  void copy_vector_from_point(std::vector<double>& destination, const space_point_t& source) const;
  void copy_vector_from_point(Eigen::Ref<Eigen::VectorXd> destination, const space_point_t& source) const;

  template <typename T, std::enable_if_t<prx::utilities::is_any_ptr<T>::value, bool> = true,  // no-lint
            typename F, std::enable_if_t<prx::utilities::is_any_ptr<F>::value, bool> = true>
  void copy(T& to, const F& from) const
  {
    copy(*to, *from);
  }

  template <typename T, std::enable_if_t<prx::utilities::is_any_ptr<T>::value, bool> = true,  // no-lint
            typename F, std::enable_if_t<not prx::utilities::is_any_ptr<F>::value, bool> = true>
  void copy(T& to, const F& from) const
  {
    copy(*to, from);
  }

  template <typename T, std::enable_if_t<not prx::utilities::is_any_ptr<T>::value, bool> = true,  // no-lint
            typename F, std::enable_if_t<prx::utilities::is_any_ptr<F>::value, bool> = true>
  void copy(T& to, const F& from) const
  {
    copy(to, *from);
  }

  template <typename T, std::enable_if_t<not prx::utilities::is_any_ptr<T>::value, bool> = true,  // no-lint
            typename F, std::enable_if_t<not prx::utilities::is_any_ptr<F>::value, bool> = true>
  void copy(T& to, const F& from) const
  {
    assert_point_dimension(to.size());
    assert_point_dimension(from.size());
    for (int i = 0; i < from.size(); ++i)
    {
      to[i] = prx::utilities::convert_to<double>(from[i]);
    }
    enforce_bounds(to);
  }

  // Explicit std::initializer_list copy is needed due to the § 14.8.2.5/5 C++11 standard
  // (std::initializer_list is a non-deduced context for a template argument)
  // No need for [T=prx::space_point_t] since we know from is initializer_list
  template <typename T, std::enable_if_t<prx::utilities::is_any_ptr<T>::value, bool> = true, typename F>
  inline void copy(T& to, const std::initializer_list<F> from) const
  {
    std::vector<F> aux_vector = from;
    copy(to, aux_vector);
  }

  template <typename F>
  inline void copy_from(const std::initializer_list<F> from)
  {
    const std::vector<F> aux_vector = from;
    copy_from(aux_vector);
  }

  template <typename T, std::enable_if_t<prx::utilities::is_any_ptr<T>::value, bool> = true>
  inline void copy_from(const T& from)
  {
    is_space_point_type(from);
    copy_from(*from);
  }

  template <typename T, std::enable_if_t<not prx::utilities::is_any_ptr<T>::value, bool> = true>
  inline void copy_from(const T& from)
  {
    assert_point_dimension(from.size());
    for (int i = 0; i < dimension; ++i)
    {
      *addresses[i] = prx::utilities::convert_to<double>(from[i]);
    }
    enforce_bounds();
  }

  template <typename To, std::enable_if_t<prx::utilities::is_any_ptr<To>::value, bool> = true>
  inline void copy_to(To& to) const
  {
    is_space_point_type(to);

    copy_to(*to);
  }

  template <typename To, std::enable_if_t<not prx::utilities::is_any_ptr<To>::value, bool> = true>
  inline void copy_to(To& to) const
  {
    assert_point_dimension(to.size());

    for (int i = 0; i < dimension; ++i)
    {
      to[i] = *addresses[i];
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

  // template <typename Point, std::enable_if_t<(prx::utils::is_any_ptr<Point>{}), bool> = true>
  template <typename Point,
            std::enable_if_t<std::is_pointer<Point>{} || prx::utilities::is_shared_ptr<Point>{}, bool> = true>
  void enforce_bounds(Point point) const
  {
    enforce_bounds(*point);
  }

  template <typename Point,
            std::enable_if_t<!(std::is_pointer<Point>{} || prx::utilities::is_shared_ptr<Point>{}), bool> = true>
  void enforce_bounds(Point& point) const
  {
    assert_point_dimension(point.size());
    for (std::size_t i = 0; i < dimension;)
    {
      if (topology[i] == topology_t::ROTATIONAL)
      {
        double& p = point[i];
        p = norm_angle_pi(p, *lower_bounds[i], *upper_bounds[i]);
        i++;
      }
      else if (topology[i] == topology_t::QUATERNION)
      {
        const double w{ point[i] };
        const double x{ point[i + 1] };
        const double y{ point[i + 2] };
        const double z{ point[i + 3] };
        Eigen::Quaterniond quat{ w, x, y, z };
        quat.normalize();
        point[i] = quat.w();
        point[i + 1] = quat.x();
        point[i + 2] = quat.y();
        point[i + 3] = quat.z();
        i += 4;
      }
      else
      {
        double& p = point[i];
        p = std::max(*lower_bounds[i], std::min(*upper_bounds[i], p));
        i++;
      }
    }
  }

  inline void enforce_bounds() const
  {
    enforce_bounds(*this);
  }

  bool satisfies_bounds(const space_point_t& point) const;
  virtual void sample(const space_point_t& point) const;

  inline std::string get_space_name() const
  {
    return space_name;
  }

  inline double& at(const std::size_t index) const
  {
    prx_assert(index < dimension, "Trying index into space at index " << index << " with dimension " << dimension);
    return *addresses[index];
  }

  inline double& operator[](const std::size_t index)
  {
    return *addresses[index];
  }

  inline double& operator[](const std::size_t index) const
  {
    return *addresses[index];
  }

  inline double get_lower_bound(const std::size_t i) const
  {
    prx_assert(i < dimension,
               "Error: Trying to get bound for " << i << " that is higher than state dimension " << dimension << ".");
    return *lower_bounds[i];
  }

  inline double get_upper_bound(const std::size_t i) const
  {
    prx_assert(i < dimension,
               "Error: Trying to get bound for " << i << " that is higher than state dimension " << dimension << ".");
    return *upper_bounds[i];
  }

  std::vector<std::pair<double, double>> get_bounds() const;

  std::vector<double> get_upper_bounds() const;

  std::vector<double> get_lower_bounds() const;

  void print_bounds() const;

  // Step the system for dt checking for the topology of each dimension
  // Implements: xt1 = xt0 + derivative * dt;
  template <typename PointIn, typename PointOut,
            std::enable_if_t<not prx::utilities::is_any_ptr<PointIn>::value, bool> = true,
            std::enable_if_t<not prx::utilities::is_any_ptr<PointOut>::value, bool> = true>
  void integrate(const PointIn& x_in, const space_t* derivative, double dt, PointOut& x_out)
  {
    assert_point_dimension(x_in.size());
    assert_point_dimension(x_out.size());
    for (unsigned i = 0; i < dimension; i++)
    {
      if (topology[i] == topology_t::EUCLIDEAN || topology[i] == topology_t::ROTATIONAL)
      {
        x_out[i] = x_in[i] + derivative->at(i) * dt;
      }
      else if (topology[i] == topology_t::DISCRETE)
      {
        x_out[i] = x_in[i] + derivative->at(i) * dt;
        x_out[i] = std::roundl(x_out[i]);
      }
      else if (topology[i] == topology_t::IDLE)
      {
        continue;
      }
      else if (topology[i] == topology_t::QUATERNION)
      {
        const double w{ x_in[i] };
        const double x{ x_in[i + 1] };
        const double y{ x_in[i + 2] };
        const double z{ x_in[i + 3] };

        const double dw{ derivative->at(i) };
        const double dx{ derivative->at(i + 1) };
        const double dy{ derivative->at(i + 2) };
        const double dz{ derivative->at(i + 3) };

        Eigen::Quaterniond quat{ w, x, y, z };
        const Eigen::Quaterniond dquat{ dw, dx, dy, dz };

        quat.coeffs() += dt * dquat.coeffs();
        quat.normalize();

        x_out[i] = quat.w();
        x_out[i + 1] = quat.x();
        x_out[i + 2] = quat.y();
        x_out[i + 3] = quat.z();
        i += 4;
      }
    }
    enforce_bounds();
  }

  // Step the system for dt checking for the topology of each dimension
  // Implements: xt1 <- xt0 + derivative * dt;
  template <typename PointIn, typename PointOut,
            std::enable_if_t<prx::utilities::is_any_ptr<PointIn>::value, bool> = true,
            std::enable_if_t<prx::utilities::is_any_ptr<PointOut>::value, bool> = true>
  inline void integrate(const PointIn& xt0, const space_t* derivative, double dt, PointOut& xt1)
  {
    integrate(*xt0, derivative, dt, *xt1);
  }

  // Step the system for dt checking for the topology of each dimension
  // Implements: xt1 <- xt0 + derivative * dt;
  template <typename PointIn, typename PointOut,
            std::enable_if_t<not prx::utilities::is_any_ptr<PointIn>::value, bool> = true,
            std::enable_if_t<prx::utilities::is_any_ptr<PointOut>::value, bool> = true>
  inline void integrate(const PointIn& xt0, const space_t* derivative, double dt, PointOut& xt1)
  {
    integrate(xt0, derivative, dt, *xt1);
  }

  // Step the system for dt checking for the topology of each dimension
  // Implements: xt1 <- xt0 + derivative * dt;
  template <typename PointIn, typename PointOut,
            std::enable_if_t<prx::utilities::is_any_ptr<PointIn>::value, bool> = true,
            std::enable_if_t<not prx::utilities::is_any_ptr<PointOut>::value, bool> = true>
  inline void integrate(const PointIn& xt0, const space_t* derivative, double dt, PointOut& xt1)
  {
    integrate(*xt0, derivative, dt, xt1);
  }

  // Integrate and store it in the state space.
  // Implements: state_space <- point + derivative * dt;
  template <typename PointIn, std::enable_if_t<not prx::utilities::is_any_ptr<PointIn>::value, bool> = true>
  void integrate(const PointIn& xt0, const space_t* derivative, double dt)
  {
    integrate(xt0, derivative, dt, *this);
  }

  // Step the system for dt checking for the topology of each dimension
  // Implements: state_space <- xt0 + derivative * dt;
  template <typename PointIn, std::enable_if_t<prx::utilities::is_any_ptr<PointIn>::value, bool> = true>
  void integrate(const PointIn& xt0, const space_t* derivative, double dt)
  {
    integrate(*xt0, derivative, dt, *this);
  }

  // Integrate the point in the state space memory, store it in the state space
  // Implements: state_space <- state_space + derivative * dt;
  void integrate(const space_t* derivative, double dt)
  {
    integrate(*this, derivative, dt, *this);
  }

  template <typename PointSrc, typename PointTarget, typename PointOut,
            std::enable_if_t<!prx::utilities::is_any_ptr<PointSrc>{}, bool> = true,
            std::enable_if_t<!prx::utilities::is_any_ptr<PointTarget>{}, bool> = true,
            std::enable_if_t<!prx::utilities::is_any_ptr<PointOut>{}, bool> = true>
  void interpolate(const PointSrc& point_src, const PointTarget& point_target, const double t, PointOut& result) const
  {
    is_space_point_type(point_src);
    is_space_point_type(point_target);
    is_space_point_type(result);
    prx_assert(0.0 <= t && t <= 1.0 + prx::constants::epsilon,
               "Interpolation requires a value between 0 and 1: given `" << t << "`");

    for (std::size_t i = 0; i < dimension;)
    {
      const double& x0_i{ point_src[i] };
      const double& x1_i{ point_target[i] };
      double& xres{ result[i] };
      if (topology[i] == topology_t::ROTATIONAL)
      {
        if (std::fabs(x0_i - x1_i) < PRX_PI)
        {
          xres = (1.0 - t) * x0_i + t * x1_i;
        }
        else
        {
          if (x0_i < x1_i)
            xres = x1_i + (1.0 - t) * (x0_i - x1_i + 2.0 * PRX_PI);
          else
            xres = x0_i + t * (2.0 * PRX_PI - x0_i + x1_i);
        }
        xres = norm_angle_pi(xres, *lower_bounds[i], *upper_bounds[i]);
        i++;
      }
      else if (topology[i] == topology_t::DISCRETE)
      {
        if (t == 1.0)
          xres = x1_i;
        else
          xres = x0_i;
        i++;
      }
      else if (topology[i] == topology_t::QUATERNION)
      {
        const double w1{ point_src[i] };
        const double x1{ point_src[i + 1] };
        const double y1{ point_src[i + 2] };
        const double z1{ point_src[i + 3] };

        const double w2{ point_target[i] };
        const double x2{ point_target[i + 1] };
        const double y2{ point_target[i + 2] };
        const double z2{ point_target[i + 3] };

        Eigen::Quaterniond quat1{ w1, x1, y1, z1 };
        const Eigen::Quaterniond quat2{ w2, x2, y2, z2 };

        const Eigen::Quaterniond q_res{ quat1.slerp(t, quat2) };

        result[i] = q_res.w();
        result[i + 1] = q_res.x();
        result[i + 2] = q_res.y();
        result[i + 3] = q_res.z();
        i += 4;
      }
      else
      {
        xres = (1.0 - t) * x0_i + t * x1_i;
        i++;
      }
    }
  }

  template <typename PointSrc, typename PointTarget, typename PointOut,
            std::enable_if_t<prx::utilities::is_any_ptr<PointSrc>{}, bool> = true,
            std::enable_if_t<prx::utilities::is_any_ptr<PointTarget>{}, bool> = true,
            std::enable_if_t<prx::utilities::is_any_ptr<PointOut>{}, bool> = true>
  inline void interpolate(const PointSrc point_src, const PointTarget point_target, const double t,
                          PointOut result) const
  {
    interpolate(*point_src, *point_target, t, *result);
  }
  // Interpolate towards point_in from the state in memory and keeping it in memory. t \in [0,1]
  template <typename PointIn, std::enable_if_t<!prx::utilities::is_any_ptr<PointIn>{}, bool> = true>
  void interpolate(const PointIn& x, const PointIn& y, double t)
  {
    interpolate(x, y, t, *this);
  }
  template <typename PointIn, std::enable_if_t<prx::utilities::is_any_ptr<PointIn>{}, bool> = true>
  void interpolate(const PointIn x, const PointIn y, double t)
  {
    interpolate(*x, *y, t, *this);
  }

  std::string print_point(const space_point_t& point, const std::size_t prec = 25) const;

  std::string print_memory(const std::size_t prec = 25) const;

  friend std::ostream& operator<<(std::ostream& os, const space_t& obj)
  {
    os << obj.print_memory(prx::constants::precision);
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

  /**
   * @brief      Gets the topology as a string in the same format as its input
   *
   * @return     A string of the topology.
   */
  std::string get_topology() const;

protected:
  space_t(const space_t* other) : dimension{ other->dimension }
  {
    for (int i = 0; i < dimension; ++i)
    {
      addresses.push_back(other->addresses[i]);
      lower_bounds.push_back(other->lower_bounds[i]);
      upper_bounds.push_back(other->upper_bounds[i]);
      topology.push_back(other->topology[i]);
    }

    space_name = other->space_name;
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

  inline void assert_point_space_name(const space_point_t& point) const
  {
    prx_assert(is_point_in_space(point),
               "Point and space have different names: " << point->_parent->space_name << " and " << space_name);
  }

  inline void assert_point_dimension(const std::size_t dim) const
  {
    prx_assert(dim == dimension, "Dimension mismatch (" << space_name << "): " << dim << " vs " << dimension);
  }

  // template <typename Point, std::enable_if_t<!std::is_any_ptr<Point>{}, bool> = true>
  template <typename Point, std::enable_if_t<!std::is_same_v<Point, space_point_t>, bool> = true>
  inline void is_space_point_type(const Point& point) const
  {
    assert_point_dimension(point.size());
  }

  template <typename Point, std::enable_if_t<std::is_same_v<Point, space_point_t>, bool> = true>
  inline void is_space_point_type(const Point point) const
  {
    // if (std::static_pointer_cast<space_snapshot_t>(point) != nullptr)
    // {
    // const space_point_t aux{ std::static_pointer_cast<space_snapshot_t>(point) };
    assert_point_space_name(point);
    // }
  }
};

typedef std::shared_ptr<space_t> space_ptr_t;
}  // namespace prx
