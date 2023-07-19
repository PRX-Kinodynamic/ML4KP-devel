#include "prx/utilities/general/param_loader.hpp"

// struct param_loader_wrap : prx::param_loader, wrapper<prx::param_loader>
// {
// 	// param_loader_wrap() : param_loader(){};
// 	// param_loader_wrap(const param_loader& other) : param_loader(other){};
// 	param_loader_wrap() : param_loader(){};
// 	param_loader_wrap(std::string file_name) : param_loader(file_name){};
// 	param_loader_wrap(std::vector<std::string> argv) : param_loader(argv){};
// 	param_loader_wrap(std::string file_name, std::vector<std::string> argv) : param_loader(file_name, argv){};
// 	param_loader_wrap(const param_loader& other) : param_loader(other){};

// 	prx::param_loader param_loader_get_item(std::string key)
// 	{
// 	    return (*this)[key];
// 	}

// 	void param_loader_set_item(std::string key, prx::param_loader other)
// 	{
// 	    (*this)[key] = other;
// 	}

// 	void print_0()
// 	{
// 		// std::cout << "print_0" << std::endl;
//         // this->get_override("print_0")();
// 		// print(params);
// 		print();
// 	}

// 	void add_opts_py(std::vector<std::string> argv)
// 	{
// 		this -> add_opts(argv);
// 	}

// };
void (prx::param_loader::*add_opts_py)(std::vector<std::string>) = &prx::param_loader::add_opts;
void (prx::param_loader::*print_0)() = &prx::param_loader::print;
// const param_loader (prx::param_loader::*operator[])(const std::string& key) = &prx::param_loader::add_opts_py;

object param_loader_get_item(prx::param_loader& pl, std::string key)
{
  return object(pl[key]);
}
void param_loader_set_item(prx::param_loader& pl, std::string key, object val)
{
  extract<int> int_extracted(val);

  if (int_extracted.check())
  {
    pl[key] = int_extracted();
    return;
  }
  extract<double> double_extracted(val);
  if (double_extracted.check())
  {
    pl[key] = double_extracted();
    return;
  }
  extract<std::string> string_extracted(val);
  if (string_extracted.check())
  {
    pl[key] = string_extracted();
    return;
  }
  extract<std::vector<int>> vec_int_extracted(val);
  if (vec_int_extracted.check())
  {
    pl[key] = vec_int_extracted();
    return;
  }
  extract<std::vector<double>> vec_double_extracted(val);
  if (vec_double_extracted.check())
  {
    pl[key] = vec_double_extracted();
    return;
  }
  extract<std::vector<std::string>> vec_string_extracted(val);
  if (vec_string_extracted.check())
  {
    pl[key] = vec_string_extracted();
    return;
  }
  // if (.check())
  // {
  // 	// Vec2& v = x();
  // 	pl[key] = ();
  // }
  prx_warn("[param_loader] Python object could not be extracted to a C++ value.")
}

void pyprx_utilities_general_param_loader()
{
  class_<prx::param_loader>("param_loader", init<>())
      // .def("__init__", make_constructor(&init_as_ptr<param_loader_wrap>(), default_call_policies()))
      .def(init<std::string>())
      // .def(init<std::vector<std::string>>())
      .def(init<std::string, std::vector<std::string>>())
      .def(init<prx::param_loader>())
      // .def("__getitem__", &param_loader_get_item)
      .def("__setitem__", &param_loader_set_item)
      // .def(init<prx::param_loader, const prx::param_loader& >, default_call_policies(), (args("other")) ))
      .def("add_file", &prx::param_loader::add_file)
      // .def("add_opts", &add_opts_py)
      .def<void (prx::param_loader::*)(std::vector<std::string>)>("add_opts", &prx::param_loader::add_opts)
      .def("get_input_path", &prx::param_loader::get_input_path)
      .def("set_input_path", &prx::param_loader::set_input_path)
      .def<const prx::param_loader (prx::param_loader::*)(const std::string& key) const>("__getitem__",
                                                                                         &prx::param_loader::operator[])
      // .def("__setitem__", &prx::param_loader::operator[])
      .def("add", &prx::param_loader::add)
      // .def("set", &prx::param_loader::set)
      // .def("print", &print_0)
      .def<void (prx::param_loader::*)()>("print", &prx::param_loader::print)
      .def("exists", &prx::param_loader::exists)
      // There should be a better way to do this...
      .def("as_string", &prx::param_loader::as<std::string>)
      .def("as_int", &prx::param_loader::as<int>)
      .def("as_bool", &prx::param_loader::as<bool>)
      .def("as_float", &prx::param_loader::as<double>)
      .def("as_float_vector", &prx::param_loader::as<std::vector<double>>)
      .def("as_string_vector", &prx::param_loader::as<std::vector<std::string>>)
      .def("as_int_vector", &prx::param_loader::as<std::vector<int>>)
      // class_<param_loader_wrap>("param_loader", init<>())
      // 	// .def("__init__", make_constructor(&init_as_ptr<param_loader_wrap>(), default_call_policies()))
      // 	.def(init<std::string>())
      // 	.def(init<std::vector<std::string>>())
      // 	.def(init<std::string, std::vector<std::string> >())
      // 	// .def(init<prx::param_loader, const prx::param_loader& >, default_call_policies(), (args("other")) ))
      // 	.def("add_file", &prx::param_loader::add_file)
      // 	.def("add_opts", &param_loader_wrap::add_opts_py)
      // 	.def("get_input_path", &prx::param_loader::get_input_path)
      // 	.def("set_input_path", &prx::param_loader::set_input_path)
      // 	.def("__getitem__", &param_loader_wrap::param_loader_get_item)
      // 	.def("__setitem__", &param_loader_wrap::param_loader_set_item)
      // 	.def("add", &prx::param_loader::add)
      // 	// .def("set", &prx::param_loader::set)
      // 	.def("print", &param_loader_wrap::print_0)
      // 	.def("exists", &prx::param_loader::exists)
      // 	// There should be a better way to do this...
      // 	.def("as_string", &prx::param_loader::as<std::string>)
      // 	.def("as_int", &prx::param_loader::as<int>)
      // 	.def("as_bool", &prx::param_loader::as<bool>)
      // 	.def("as_float", &prx::param_loader::as<double>)
      // 	.def("as_float_vector", &prx::param_loader::as<std::vector<double>>)
      // 	.def("as_string_vector", &prx::param_loader::as<std::vector<std::string>>)
      // 	.def("as_int_vector", &prx::param_loader::as<std::vector<int>>)
      // .def("", &prx::param_loader::)
      // .def("", &prx::param_loader::)
      // .def("", &prx::param_loader::)
      // .def("", &prx::param_loader::)
      // .def("", &prx::param_loader::)
      // .def("", &prx::param_loader::)
      ;
}