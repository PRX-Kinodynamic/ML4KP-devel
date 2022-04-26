#define BOOST_AUTO_TEST_MAIN math_functions_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/math/math_functions.hpp"

void test1()
{
	// values taken from wolframAlpha: jacobian of (4x^2y, x-y^2)
	auto res_jac = [](Eigen::VectorXd m)
	{
		double x = m[0];
		double y = m[1];
		Eigen::MatrixXd res(2,2);

		res(0,0) = 8 * x * y;
		res(0,1) = 4 * x * x;
		res(1,0) = 1;
		res(1,1) = -2 * y;
		return res;
	};

	std::function<Eigen::VectorXd(Eigen::VectorXd)> f = [](Eigen::VectorXd m)
	{
		double x = m[0];
		double y = m[1];
		Eigen::VectorXd res(2);
		res[0] = 4 * x * x * y;
		res[1] = x - y * y;
		return res;
	};
	
	Eigen::VectorXd m(2);
	
	m[0] = 0;	m[1] = 0;
	auto r1_c = res_jac(m);
	auto r1_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = 0.5;	m[1] = 0.5;
	auto r2_c = res_jac(m);
	auto r2_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = -0.5;	m[1] = -0.5;
	auto r3_c = res_jac(m);
	auto r3_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = 0.5;	m[1] = -0.5;
	auto r4_c = res_jac(m);
	auto r4_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = -0.5;	m[1] = 0.5;
	auto r5_c = res_jac(m);
	auto r5_p = prx::math_functions::differentiate(f, m, 0.01);

	std::cout << "Correct: "  << r5_c << std::endl;
	std::cout << "Computed: " << r5_p << std::endl;

	BOOST_CHECK(prx::are_matrices_approx_equal(r1_c, r1_p));
	BOOST_CHECK(prx::are_matrices_approx_equal(r2_c, r2_p));
	BOOST_CHECK(prx::are_matrices_approx_equal(r3_c, r3_p));
	BOOST_CHECK(prx::are_matrices_approx_equal(r4_c, r4_p));
	BOOST_CHECK(prx::are_matrices_approx_equal(r5_c, r5_p));

	printf("%s\t[ OK ]\n", __PRETTY_FUNCTION__ );

}

void test2()
{
	// values taken from wikipedia: https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant - Examples (4)
	// jacobian of:
	// y_1 = x_1
	// y_2 = 5 * x_3
	// y_3 = 4 * ( x_2 ) ^ 2 - 2 * x_3
	// y_4 = x_3 * sin(x_1)
	auto res_jac = [](Eigen::VectorXd m)
	{
		double x1 = m[0];
		double x2 = m[1];
		double x3 = m[2];
		Eigen::MatrixXd res(4,3);

		res(0,0) = 1;
		res(0,1) = 0;
		res(0,2) = 0;
		res(1,0) = 0;
		res(1,1) = 0;
		res(1,2) = 5;
		res(2,0) = 0;
		res(2,1) = 8 * x2;
		res(2,2) = -2;
		res(3,0) = x3 * std::cos(x1);
		res(3,1) = 0;
		res(3,2) = std::sin(x1);
		return res;
	};

	std::function<Eigen::VectorXd(Eigen::VectorXd)> f = [](Eigen::VectorXd m)
	{
		double x_1 = m[0];
		double x_2 = m[1];
		double x_3 = m[2];

		Eigen::VectorXd res(4);
		res[0] = x_1;
		res[1] = 5 * x_3;
		res[2] = 4 * std::pow( x_2, 2) - 2 * x_3;
		res[3] = x_3 * std::sin(x_1);

		return res;
	};
	
	Eigen::VectorXd m(3);
	
	m[0] = 0;	m[1] = 0; m[2] = 0;
	auto r1_c = res_jac(m);
	auto r1_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = 0.5;	m[1] = 0.5; m[2] = 0.5;
	auto r2_c = res_jac(m);
	auto r2_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = -0.5;	m[1] = -0.5; m[2] = -0.5;
	auto r3_c = res_jac(m);
	auto r3_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = 0.5;	m[1] = -0.5;	m[2] = 0.5;
	auto r4_c = res_jac(m);
	auto r4_p = prx::math_functions::differentiate(f, m, 0.01);

	m[0] = -0.5;	m[1] = 0.5;	m[2] = -0.5;
	auto r5_c = res_jac(m);
	auto r5_p = prx::math_functions::differentiate(f, m, 0.01);

	// std::cout << "Correct:\n"  << r1_c << std::endl;
	// std::cout << "Computed:\n" << r1_p << std::endl;
	BOOST_CHECK(prx::are_matrices_approx_equal(r1_c, r1_p, 0.01));
	BOOST_CHECK(prx::are_matrices_approx_equal(r2_c, r2_p, 0.01));
	BOOST_CHECK(prx::are_matrices_approx_equal(r3_c, r3_p, 0.01));
	BOOST_CHECK(prx::are_matrices_approx_equal(r4_c, r4_p, 0.01));
	BOOST_CHECK(prx::are_matrices_approx_equal(r5_c, r5_p, 0.01));

	printf("%s\t[ OK ]\n", __PRETTY_FUNCTION__ );
}


BOOST_AUTO_TEST_CASE( math_functions_test )
{   
	test1(); 
	test2();

}