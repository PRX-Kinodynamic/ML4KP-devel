#include <chrono>
#include <thread>

#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/string_manip.hpp"
#include "prx/utilities/general/constants.hpp"

namespace prx
{
namespace utilities
{

class base_value_wrapper_t
{
public:
  virtual std::string log() const = 0;

  virtual ~base_value_wrapper_t(){};

  // protected:
  base_value_wrapper_t() = default;
  base_value_wrapper_t(const base_value_wrapper_t&) = default;
};

template <typename T>
class generic_value_wrapper_t : public base_value_wrapper_t
{
public:
  generic_value_wrapper_t(const T value) : base_value_wrapper_t(), _wrapped_value(value)
  {
  }
  virtual ~generic_value_wrapper_t()
  {
  }

  virtual std::string log() const override
  {
    std::stringstream ss;
    ss << *_wrapped_value;
    return ss.str();
  }

protected:
  T _wrapped_value;
};

template <typename F>
class generic_function_wrapper_t : public base_value_wrapper_t
{
public:
  generic_function_wrapper_t(const F func) : base_value_wrapper_t(), _function(func)
  {
  }
  virtual ~generic_function_wrapper_t()
  {
  }

  virtual std::string log() const override
  {
    std::stringstream ss;
    ss << _function();
    return ss.str();
  }

protected:
  // T _wrapped_value;
  F _function;
};

class timed_logger_t
{
public:
  timed_logger_t() = delete;

  template <typename Duration>
  timed_logger_t(const Duration& duration, std::ostream& ostream, const std::string sep = " ")
    : _duration(duration), _log_stream(ostream.rdbuf()), _sep(sep)
  {
  }
  ~timed_logger_t()
  {
  }

  void run()
  {
    std::thread t1(&timed_logger_t::run_thread, this);
    _keep_logging = true;
    _thread.swap(t1);
  }

  void stop()
  {
    _keep_logging = false;
    _thread.join();
  }

  template <typename F>
  void add(const std::string& msg, F& f)
  {
    strs_to_log.push_back(msg);
    // auto f3 = std::bind(f, args...);
    values_to_log.push_back(std::make_unique<generic_function_wrapper_t<decltype(f)>>(f));
  }

  template <class T>
  void add(const std::string& msg, const T* ptr)
  {
    strs_to_log.push_back(msg);
    values_to_log.push_back(std::make_unique<generic_value_wrapper_t<const T*>>(ptr));
  }

private:
  void run_thread()
  {
    while (_keep_logging)
    {
      for (int i = 0; i < values_to_log.size(); ++i)
      {
        _log_stream << strs_to_log[i] << _sep;
        _log_stream << values_to_log[i]->log();
        _log_stream << std::endl;  // Maybe use '\n'
      }
      std::this_thread::sleep_for(_duration);
    }
  }

  std::ostream _log_stream;

  std::string _sep;

  std::chrono::milliseconds _duration;
  std::thread _thread;
  bool _keep_logging;
  std::vector<std::string> strs_to_log;
  std::vector<std::unique_ptr<base_value_wrapper_t>> values_to_log;
};

}  // namespace utilities
}  // namespace prx