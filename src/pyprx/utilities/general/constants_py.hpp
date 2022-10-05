#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/general/constants.hpp"

using namespace boost::python;


void pyprx_utilities_general_constants()
{
	// Var("EPSILON", PRX_EPSILON);
	scope().attr("PRX_EPSILON") = PRX_EPSILON;
	scope().attr("PRX_INFINITY") = PRX_INFINITY;
	scope().attr("PRX_PI") = PRX_PI;
	scope().attr("lib_path")    = prx::lib_path;
	scope().attr("models_path") = prx::models_path;
	scope().attr("input_path")  = prx::input_path;
	scope().attr("js_path") 	= prx::js_path;
	scope().attr("out_path") 	= prx::out_path;

}