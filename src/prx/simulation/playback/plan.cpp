#include "prx/simulation/playback/plan.hpp"

namespace prx
{
plan_t::plan_t(const space_t* new_space)
{
  control_space = new_space;
  steps.push_back(plan_step_t(control_space->make_point(), 0));
  num_steps = 0;
  max_num_steps = 1;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
}

plan_t::plan_t(const plan_t& other)
{
  control_space = other.control_space;
  num_steps = 0;
  max_num_steps = 0;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  (*this) = other;
}

plan_t::~plan_t()
{
  steps.clear();
}

void plan_t::resize(unsigned num_size)
{
  while (max_num_steps < num_size)
  {
    increase_buffer();
  }
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  std::advance(end_iterator, num_steps);
  std::advance(const_end_iterator, num_steps);
}

plan_t& plan_t::operator=(const plan_t& t)
{
  prx_assert(control_space->get_space_name() == t.control_space->get_space_name(),
             "Assigning a plan that doesn't have a matching control space.");

  if (this != &t)
  {
    // check the size of the two plans
    while (max_num_steps < t.num_steps)
    {
      increase_buffer();
    }
    // clear the plan
    clear();

    for (auto&& step : t)
    {
      control_space->copy_point((*end_iterator).control, step.control);
      (*end_iterator).duration = step.duration;
      ++num_steps;
      ++end_iterator;
      ++const_end_iterator;
    }
  }
  return *this;
}

plan_t& plan_t::operator+=(const plan_t& t)
{
  unsigned new_size = t.num_steps + num_steps;
  // check the size of the two plans
  while (max_num_steps < new_size)
  {
    increase_buffer();
    end_iterator = steps.begin();
    const_end_iterator = steps.begin();
    std::advance(end_iterator, num_steps);
    std::advance(const_end_iterator, num_steps);
  }

  for (auto&& step : t)
  {
    control_space->copy_point((*end_iterator).control, step.control);
    (*end_iterator).duration = step.duration;
    ++num_steps;
    ++end_iterator;
    ++const_end_iterator;
  }
  return *this;
}

void plan_t::clear()
{
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  num_steps = 0;
}

void plan_t::copy_to(const double start_time, const double duration, plan_t& t)
{
  prx_assert(control_space->get_space_name() == t.control_space->get_space_name(),
             "Copying a plan that doesn't have a matching control space.");
  double s = start_time;
  double d = duration;
  auto step = steps.begin();
  while (s > step->duration)
  {
    s -= step->duration;
    step++;
    if (step == steps.end())
      prx_throw("Indexed into plan with time outside the plan's full duration.");
  }
  if (d < step->duration)
  {
    t.copy_onto_back(step->control, d);
    // std::cout<<duration<<" Copy to: "<<t.duration()<<std::endl;
    return;
  }
  t.copy_onto_back(step->control, step->duration - s);
  d -= (step->duration - s);
  step++;
  while (step != steps.end())
  {
    if (d < step->duration)
    {
      t.copy_onto_back(step->control, d);
      break;
    }
    t.copy_onto_back(step->control, step->duration);
    d -= step->duration;
    step++;
  }
  // std::cout<<duration<<" Copy to: "<<t.duration()<<std::endl;
}

void plan_t::copy_onto_back(Eigen::VectorXd v_control, double time)
{
  // PRX_DEPRECIATED;
  control_space->copy_from(v_control);
  append_onto_back(time, true);
}

void plan_t::copy_onto_back(space_point_t control, double time)
{
  if ((num_steps + 1) >= max_num_steps)
  {
    increase_buffer();
    end_iterator = steps.begin();
    const_end_iterator = steps.begin();
    std::advance(end_iterator, num_steps);
    std::advance(const_end_iterator, num_steps);
  }
  control_space->copy_point((*end_iterator).control, control);
  (*end_iterator).duration = time;
  ++end_iterator;
  ++const_end_iterator;
  ++num_steps;
}

void plan_t::copy_onto_front(space_point_t control, double time)
{
  if ((num_steps + 1) >= max_num_steps)
    increase_buffer();

  plan_step_t new_step = steps.back();
  steps.pop_back();
  steps.push_front(new_step);
  control_space->copy_point((*steps.begin()).control, control);
  (*steps.begin()).duration = time;
  ++num_steps;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  std::advance(end_iterator, num_steps);
  std::advance(const_end_iterator, num_steps);
}

void plan_t::append_onto_front(double time)
{
  if ((num_steps + 1) >= max_num_steps)
    increase_buffer();

  plan_step_t new_step = steps.back();
  steps.pop_back();
  steps.push_front(new_step);
  (*steps.begin()).duration = time;
  ++num_steps;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  std::advance(end_iterator, num_steps);
  std::advance(const_end_iterator, num_steps);
}

void plan_t::append_onto_back(double time, bool copy_from_control_space)
{
  if ((num_steps + 1) >= max_num_steps)
  {
    increase_buffer();
    end_iterator = steps.begin();
    const_end_iterator = steps.begin();
    std::advance(end_iterator, num_steps);
    std::advance(const_end_iterator, num_steps);
  }
  (*end_iterator).duration = time;
  if (copy_from_control_space)
    control_space->copy_to_point((*end_iterator).control);

  ++end_iterator;
  ++const_end_iterator;
  ++num_steps;
}

void plan_t::append_onto_back(double time, space_t* ctrl_space)
{
  PRX_DEPRECIATED_1("Use append_onto_back(time, true) instead")
  append_onto_back(time);
  ctrl_space->copy_to_point(this->back().control);
}

void plan_t::extend_last_control(double time)
{
  prx_assert(num_steps > 0, "Can't extend the last control if a plan has no controls");
  this->back().duration += time;
}

void plan_t::pop_front()
{
  if (num_steps == 0)
    prx_throw("Trying to pop the front off an empty plan.");
  plan_step_t popped = steps[0];
  for (unsigned i = 0; i < num_steps - 1; ++i)
  {
    steps[i] = steps[i + 1];
  }
  num_steps--;
  steps[num_steps] = popped;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  std::advance(end_iterator, num_steps);
  std::advance(const_end_iterator, num_steps);
}

void plan_t::pop_back()
{
  if (num_steps == 0)
    prx_throw("Trying to pop the backs off an empty plan.");
  num_steps--;
  end_iterator = steps.begin();
  const_end_iterator = steps.begin();
  std::advance(end_iterator, num_steps);
  std::advance(const_end_iterator, num_steps);
}

std::string plan_t::print(unsigned precision) const
{
  std::stringstream out(std::stringstream::out);
  // out << "\n";
  for (const plan_step_t& step : *this)
  {
    out << "[" << control_space->print_point(step.control, precision) << ", " << step.duration << "s]";
  }
  return out.str();
}

void plan_t::increase_buffer()
{
  if (max_num_steps != 0)
  {
    for (unsigned i = 0; i < max_num_steps; i++)
    {
      steps.push_back(plan_step_t(control_space->make_point(), 0));
    }
    max_num_steps *= 2;
  }
  else
  {
    steps.push_back(plan_step_t(control_space->make_point(), 0));
    max_num_steps = 1;
    num_steps = 0;
    end_iterator = steps.begin();
    const_end_iterator = steps.begin();
  }
}

void plan_t::expand()
{
  plan_t expanded_plan(control_space);
  for (auto step : steps)
  {
    for (double t = 0; t < step.duration; t += simulation_step)
    {
      expanded_plan.copy_onto_back(step.control, simulation_step);
    }
  }
  (*this) = expanded_plan;
}

void plan_t::compress()
{
  plan_t compressed_plan(control_space);

  for (auto step : steps)
  {
    if (compressed_plan.size() == 0)
    {
      compressed_plan.copy_onto_back(step.control, step.duration);
    }
    else if (control_space->equal_points(compressed_plan.back().control, step.control))
    {
      compressed_plan.extend_last_control(step.duration);
    }
    else
    {
      compressed_plan.copy_onto_back(step.control, step.duration);
    }
  }
  (*this) = compressed_plan;
}

void plan_t::to_file(const std::string file_name, const std::ios_base::openmode _mode) const
{
  std::ofstream ofs_map;
  ofs_map.open(file_name.c_str(), _mode);

  for (unsigned i = 0; i < num_steps; ++i)
  {
    ofs_map << steps[i].duration << prx::separating_value;
    ofs_map << steps[i].control;
    ofs_map << "\n";
  }

  ofs_map.close();
}

void plan_t::from_file(const std::string file_name)
{
  const char sep{ prx::separating_value };
  std::ifstream ifs(file_name);
  std::string line;

  space_point_t aux = control_space->make_point();
  double time;
  while (std::getline(ifs, line))
  {
    std::istringstream ss(line);
    std::string token;
    int i = 0;
    std::string ctrl = "";
    while (std::getline(ss, token, sep))
    {
      if (i == 0)
        time = std::stod(token);
      else
        ctrl += token + sep;
      i++;
    }
    // std::cout << "ctrl:" << ctrl << std::endl;
    control_space->copy_point_from_string(aux, ctrl, sep);
    copy_onto_back(aux, time);
  }
}
}  // namespace prx
