#include "prx/utilities/spaces/space.hpp"

#include <limits>
#include <sstream>
#include <iomanip>

namespace prx
{

space_t::space_t(const std::string& topo, const std::vector<double*>& in_addresses, const std::string& name)
  : dimension(in_addresses.size())
{
  owned_values = true;
  addresses.clear();
  for (double* v : in_addresses)
  {
    addresses.push_back(v);
  }
  prx_assert(topo.size() == addresses.size(),
             "Topology " << topo << " does not match addresses size " << addresses.size());

  topology.clear();
  lower_bounds.clear();
  upper_bounds.clear();
  for (auto c : topo)
  {
    switch (c)
    {
      case 'E':
        topology.push_back(topology_t::EUCLIDEAN);
        lower_bounds.push_back(new double(std::numeric_limits<double>::lowest()));
        upper_bounds.push_back(new double(std::numeric_limits<double>::max()));
        break;
      case 'R':
        topology.push_back(topology_t::ROTATIONAL);
        lower_bounds.push_back(new double(-PRX_PI));
        upper_bounds.push_back(new double(PRX_PI));
        break;
      case 'D':
        topology.push_back(topology_t::DISCRETE);
        lower_bounds.push_back(new double(std::numeric_limits<int>::lowest()));
        upper_bounds.push_back(new double(std::numeric_limits<int>::max()));
        break;
      case 'I':
        topology.push_back(topology_t::IDLE);
        lower_bounds.push_back(new double(std::numeric_limits<int>::lowest()));
        upper_bounds.push_back(new double(std::numeric_limits<int>::max()));
        break;
      case 'Q':
        topology.push_back(topology_t::QUATERNION);
        lower_bounds.push_back(new double(-1.0));
        upper_bounds.push_back(new double(1.0));
        break;
      default:
        prx_throw("Bad topology identifier '" << c << "' from topology string " << topo);
    }
  }
  space_name = name;
  enforce_bounds();
}

space_t::space_t(const std::string& topology, const std::vector<double*>& addresses)
  : space_t(topology, addresses, topology)
{
}

space_t::space_t(const std::vector<const space_t*>& spaces) : dimension{ 0 }
{
  owned_values = false;
  addresses.clear();
  topology.clear();
  space_name = "";

  for (auto space : spaces)
  {
    if (space != nullptr && space->addresses.size() != 0)
    {
      for (auto val : space->addresses)
      {
        addresses.push_back(val);
      }
      for (auto val : space->topology)
      {
        topology.push_back(val);
      }
      for (auto val : space->lower_bounds)
      {
        lower_bounds.push_back(val);
      }
      for (auto val : space->upper_bounds)
      {
        upper_bounds.push_back(val);
      }
      if (!space_name.empty())
        space_name += "|";
      space_name += space->space_name;
    }
  }
  dimension = addresses.size();
}

space_t::~space_t()
{
  // std::cout << "space_name: " << space_name << std::endl;
  // PRX_DEBUG_PRINT
  if (owned_values)
  {
    for (auto d : lower_bounds)
    {
      delete d;
    }
    for (auto d : upper_bounds)
    {
      delete d;
    }
  }
  // PRX_DEBUG_PRINT
}

void space_t::set_bounds(const std::vector<double>& lower, const std::vector<double>& upper)
{
  prx_assert(lower.size() == lower_bounds.size(), "Given lower bounds have size "
                                                      << lower.size() << " while the space bounds have size "
                                                      << lower_bounds.size());
  prx_assert(upper.size() == upper_bounds.size(), "Given upper bounds have size "
                                                      << upper.size() << " while the space bounds have size "
                                                      << upper_bounds.size());

  for (unsigned i = 0; i < dimension; ++i)
  {
    prx_assert(lower[i] <= upper[i],
               "Index " << i << " has lower bound (" << lower[i] << ") greater than upper bound(" << upper[i] << ")");
    *(lower_bounds[i]) = lower[i];
    *(upper_bounds[i]) = upper[i];
  }
}

space_point_t space_t::make_point() const
{
  const std::size_t dimension{ this->size() };
  return std::shared_ptr<space_snapshot_t>(new space_snapshot_t(this, dimension));
  // return std::make_shared<space_snapshot_t>(this, size());
}

space_point_t space_t::clone_point(const space_point_t& point) const
{
  assert_point_space_name(point);

  space_point_t new_point{ this->make_point() };
  this->copy(new_point, point);
  return new_point;
}

void space_t::point_union(const space_point_t& p1, const space_point_t& p2, const space_point_t& pu)
{
  prx_assert((p1->_parent->space_name + "|" + p2->_parent->space_name) == space_name,
             "Points and space have different names: " << p1->_parent->space_name << "|" << p2->_parent->space_name
                                                       << " != " << space_name);
  prx_assert(p1->get_dim() + p2->get_dim() == dimension,
             "Points dimension must add to this space dimension: " << p1->get_dim() << " + " << p2->get_dim()
                                                                   << " != " << dimension);

  // auto new_point = make_point();

  for (unsigned i = 0; i < p1->get_dim(); ++i)
  {
    pu->_memory[i] = p1->_memory[i];
  }

  for (unsigned i = 0; i < p2->get_dim(); ++i)
  {
    pu->_memory[p1->get_dim() + i] = p2->_memory[i];
  }
  // return new_point;
}

void space_t::split_point(const space_point_t& ps_to_split, const space_point_t& ps1, const space_point_t& ps2)
{
  prx_assert((ps1->_parent->space_name + "|" + ps2->_parent->space_name) == ps_to_split->_parent->space_name,
             "Different spaces: " << ps1->_parent->space_name + "|" + ps2->_parent->space_name
                                  << "!=" << ps_to_split->_parent->space_name);

  for (int i = 0; i < ps1->get_dim(); ++i)
  {
    ps1->_memory[i] = ps_to_split->_memory[i];
  }

  for (int i = 0; i < ps2->get_dim(); ++i)
  {
    ps2->_memory[i] = ps_to_split->_memory[ps1->get_dim() + i];
  }
}

bool space_t::equal_points(const space_point_t& point1, const space_point_t& point2) const
{
  assert_point_space_name(point1);
  assert_point_space_name(point2);

  for (unsigned i = 0; i < dimension; ++i)
  {
    if (fabs(point1->_memory[i] - point2->_memory[i]) > PRX_EPSILON)
    {
      return false;
    }
  }
  return true;
}

void space_t::copy_to_vector(Eigen::VectorXd& vector) const
{
  PRX_DEPRECATED("Use 'space_t::copy_to' instead");
  this->copy_to(vector);
}

void space_t::copy_from_vector(const Eigen::VectorXd& vector)
{
  PRX_DEPRECATED("Use 'space_t::copy_from' instead");
  this->copy_from(vector);
}

void space_t::copy_to_point(const space_point_t point) const
{
  PRX_DEPRECATED("Use 'space_t::copy_to' instead");
  this->copy_to(point);
}
void space_t::copy_from_point(const space_point_t point) const
{
  PRX_DEPRECATED("Use 'space_t::copy_from' instead");
  // NOTE: if this point come from a different space with the same name, different bounds may be in effect. Consider
  // adding bounds enforcement here.
  prx_assert(point->_parent->space_name == space_name,
             "Point and space have different names: " << point->_parent->space_name << " and " << space_name);
  for (unsigned i = 0; i < dimension; ++i)
  {
    *addresses[i] = point->_memory[i];
  }
}

void space_t::copy_point(const space_point_t destination, const space_point_t source) const
{
  PRX_DEPRECATED("Use 'space_t::copy' instead");
  prx_assert(destination->_parent->space_name == source->_parent->space_name,
             "Points have different parent spaces: " << destination->_parent->space_name << " and "
                                                     << source->_parent->space_name);
  prx_assert(destination->_parent->space_name == space_name,
             "Points and space have different names: " << destination->_parent->space_name << " and " << space_name);
  for (unsigned i = 0; i < dimension; ++i)
  {
    destination->_memory[i] = source->_memory[i];
  }
}

void space_t::copy_to_vector(std::vector<double>& destination) const
{
  PRX_DEPRECATED("Use 'space_t::copy_to' instead");
  destination.clear();
  for (unsigned i = 0; i < dimension; ++i)
  {
    destination.push_back(*addresses[i]);
  }
}

void space_t::copy_from_vector(const std::vector<double>& source)
{
  PRX_DEPRECATED("Use 'space_t::copy_from' instead");
  this->copy_from(source);
}

void space_t::copy_point_from_vector(space_point_t destination, const std::vector<double>& source) const
{
  PRX_DEPRECATED("Use 'space_t::copy' instead");
  copy(destination, source);
}

void space_t::copy_point_from_vector(space_point_t destination, Eigen::Ref<Eigen::VectorXd> source) const
{
  PRX_DEPRECATED("Use 'space_t::copy' instead");

  prx_assert(destination->_parent->dimension == source.size(),
             "Point and vector have different sizes: " << destination->_parent->dimension << " and " << source.size());
  for (unsigned i = 0; i < dimension; ++i)
  {
    destination->_memory[i] = source[i];
  }
  enforce_bounds(destination);
}

void space_t::copy_vector_from_point(std::vector<double>& destination, const space_point_t& source) const
{
  assert_point_space_name(source);

  for (unsigned i = 0; i < dimension; ++i)
  {
    destination.push_back(source->_memory[i]);
  }
}

void space_t::copy_vector_from_point(Eigen::Ref<Eigen::VectorXd> destination, const space_point_t& source) const
{
  assert_point_space_name(source);
  assert_point_dimension(destination.size());

  for (unsigned i = 0; i < dimension; ++i)
  {
    destination[i] = source->_memory[i];
  }
}

bool space_t::satisfies_bounds(const space_point_t& point) const
{
  assert_point_space_name(point);

  for (unsigned i = 0; i < dimension; i++)
  {
    double& p = point->_memory[i];
    if (p < *lower_bounds[i])
    {
      return false;
    }
    if (p > *upper_bounds[i])
    {
      return false;
    }
  }
  return true;
}
void space_t::sample(const space_point_t& point) const
{
  prx_assert(point->_parent->space_name == space_name,
             "Point and space have different names: " << point->_parent->space_name << " and " << space_name);
  unsigned i = 0;
  while (i < dimension)
  {
    if (topology[i] == topology_t::QUATERNION)
    {
      quaternion_t quat = Eigen::Quaterniond::UnitRandom();
      point->_memory[i] = quat.w();
      point->_memory[i + 1] = quat.x();
      point->_memory[i + 2] = quat.y();
      point->_memory[i + 3] = quat.z();
      i += 4;
    }
    else
    {
      point->_memory[i] = uniform_random(*lower_bounds[i], *upper_bounds[i]);
      if (topology[i] == topology_t::DISCRETE)
        point->_memory[i] = round(point->_memory[i]);
      i++;
    }
  }
}

std::vector<std::pair<double, double>> space_t::get_bounds() const
{
  std::vector<std::pair<double, double>> bounds;
  for (unsigned i = 0; i < dimension; i++)
  {
    bounds.push_back(std::make_pair(*lower_bounds[i], *upper_bounds[i]));
  }
  return bounds;
}

std::vector<double> space_t::get_upper_bounds() const
{
  std::vector<double> ub;
  for (unsigned i = 0; i < dimension; i++)
  {
    ub.push_back(*upper_bounds[i]);
  }
  return ub;
}

std::vector<double> space_t::get_lower_bounds() const
{
  std::vector<double> ub;
  for (unsigned i = 0; i < dimension; i++)
  {
    ub.push_back(*lower_bounds[i]);
  }
  return ub;
}

void space_t::print_bounds() const
{
  std::cout << "Bounds: ( ";
  for (int i = 0; i < dimension; ++i)
  {
    if (i != 0)
      std::cout << ", ";
    std::cout << *lower_bounds[i];
  }
  std::cout << " ), (";
  for (int i = 0; i < dimension; ++i)
  {
    if (i != 0)
      std::cout << ", ";
    std::cout << *upper_bounds[i];
  }
  std::cout << ")" << std::endl;
}

std::string space_t::print_point(const space_point_t& point, const std::size_t prec) const
{
  assert_point_space_name(point);

  std::stringstream out(std::stringstream::out);
  if (dimension > 0)
  {
    // out << std::fixed << std::setprecision(prec) << '<';
    out << std::fixed << std::setprecision(prec);
    for (unsigned i = 0; i < dimension - 1; ++i)
      out << point->_memory[i] << ',';
    out << point->_memory[dimension - 1];
  }

  return out.str();
}

std::string space_t::print_memory(const std::size_t prec) const
{
  std::stringstream out(std::stringstream::out);
  if (dimension > 0)
  {
    out << std::fixed << std::setprecision(prec) << '<';
    for (unsigned i = 0; i < dimension - 1; ++i)
      out << *addresses[i] << ',';
    out << *addresses[dimension - 1] << '>';
  }

  return out.str();
}

std::string space_t::get_topology() const
{
  std::stringstream ss;
  for (auto topo : topology)
  {
    switch (topo)
    {
      case topology_t::EUCLIDEAN:
        ss << "E";
        break;
      case topology_t::ROTATIONAL:
        ss << "R";
        break;
      case topology_t::DISCRETE:
        ss << "D";
        break;
      case topology_t::IDLE:
        ss << "I";
        break;
      case topology_t::QUATERNION:
        ss << "Q";  // Should this be Qqqq?
        break;
      default:
        ss << "ERROR";
    }
  }
  return ss.str();
}
}  // namespace prx
