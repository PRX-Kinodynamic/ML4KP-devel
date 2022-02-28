#define BOOST_AUTO_TEST_MAIN care_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"


BOOST_AUTO_TEST_CASE( care_test )
{   
    Eigen::MatrixXd a(2,2);
    a << 4, 3, -4.5, -3.5;

    Eigen::MatrixXd b(2,1);
    b << 1, -1;

    Eigen::MatrixXd q(2,2);
    q << 9, 6, 6, 4.;

    auto r = Eigen::MatrixXd::Identity(1,1);
    auto x = prx::care::solve(a, b, q, r);

    std::cout << "x: " << x << std::endl;
    // >>> x
    // array([[ 21.72792206,  14.48528137],
    //        [ 14.48528137,   9.65685425]])
    // >>> np.allclose(a.T.dot(x) + x.dot(a)-x.dot(b).dot(b.T).dot(x), -q)
    BOOST_CHECK(prx::are_approx_equal(x(0,0), 21.72792206, 1e-5));
    BOOST_CHECK(prx::are_approx_equal(x(0,1), 14.48528137, 1e-5));
    BOOST_CHECK(prx::are_approx_equal(x(1,0), 14.48528137, 1e-5));
    BOOST_CHECK(prx::are_approx_equal(x(1,1),  9.65685425, 1e-5));
   
    

}
