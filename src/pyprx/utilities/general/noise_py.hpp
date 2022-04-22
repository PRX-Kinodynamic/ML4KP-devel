#include "prx/utilities/general/noise.hpp"

using namespace boost::python;


template <class T>
void add_noise_space_point_wrapper_3(T& noise, const prx::space_point_t& pt, unsigned int start, unsigned int end)
{
	noise.add_noise(pt, start, end);
}

template <class T>
void add_noise_space_point_wrapper_2(T& noise, const prx::space_point_t& pt, unsigned int start)
{
	noise.add_noise(pt, start);
}

template <class T>
void add_noise_space_point_wrapper_1(T& noise, const prx::space_point_t& pt)
{
	noise.add_noise(pt);
}

template <class T, class S>
S add_noise_S_wrapper(T& noise, S v)
{
	return noise.add_noise(v);
}

// template <class T, class S>
// void add_noise_vector_S_wrapper(T noise, std::vector<S> v)
// {
// 	noise.add_noise(v);
// }


template <class T, class... Types >
void bind_noise(const std::string& name)
{
	// void (T::*add_noise_space_point_wrapper_2)(const prx::space_point_t& pt, unsigned int start) = &T::add_noise;
	void (T::*add_noise_space_point_wrapper)(const prx::space_point_t& pt, unsigned int start, unsigned int end) = &T::add_noise;
	// void (T::*add_noise_space_point_wrapper_2)(const prx::space_point_t& pt, unsigned int start) = &T::add_noise;

	// BOOST_PYTHON_FUNCTION_OVERLOADS(noise_add_noise_overloads, add_noise_space_point_wrapper, 1, 2);
	// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(noise_add_noise_overloads, T::add_noise, 2, 3);

    class_<T, std::shared_ptr<T>>(name.c_str(), init<>() )
		.def("__init__", make_constructor(&init_as_ptr<T, Types...>, default_call_policies()))
		// .def("add_noise", add_noise_space_point_wrapper)
		.def("add_noise", add_noise_space_point_wrapper_1<T>)
		.def("add_noise", add_noise_space_point_wrapper_2<T>)
		.def("add_noise", add_noise_space_point_wrapper_3<T>)
		.def("add_noise", add_noise_S_wrapper<T, double>)
		// .def("add_noise", add_noise_S_wrapper<T, float>)
		// .def("add_noise", add_noise_S_wrapper<T, std::vector,double>)
		// .def("add_noise", add_noise_S_wrapper<T, Eigen::MatrixXd>)
		// .def("add_noise", &T::add_noise, noise_add_noise_overloads())
		// .def("add_noise", add_noise_space_point_wrapper_2)
    	;

}

void pyprx_utilities_general_noise()
{
	bind_noise<prx::gaussian_noise_t, double, double>("gaussian_noise");
	bind_noise<prx::uniform_noise_t, double, double>("uniform_noise");
}
