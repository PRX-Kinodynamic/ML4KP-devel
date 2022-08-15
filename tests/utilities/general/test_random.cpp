#define BOOST_AUTO_TEST_MAIN test_random
#include "prx/utilities/general/statistics.hpp"
#include <boost/test/unit_test.hpp>
#include <string>
#include <algorithm>

// IF TEST FAILS:
// 		This implements a statistical test ==> there is a low, non-zero probability this could fail even for a correct
// 		implementation of the tested methods. So, if this test fails: run again (effectively changing the seed)
// 		multiple times to be sure that the failure was the rare case of expected failure.
//
//  Some comments on these statistical tests on random distributions:
//  	* According to google's tests (https://abseil.io/resources/swe-book/html/toc.html) unit tests
//  	  should be simple, no fors, no complex logic, yadayadayada. This does not apply to some
//  	  type of implementation such as random functions.
//  	* Some approches could be:
//  		* Get n samples and check bounds --> How big n? The test could pass for a bad random implementation
//  			(i.e. return ( SOME_CONSTANT % upper_bound ) + lower_bound )
//  		* Graphical test (get n samples and plot) --> needs human inspection, not really a unit test.
//  	* Instead, use a statistical test as proposed by Knuth in Seminumerical Algorithms pages 48-52
//  		(third edition)
//  	* More in depth analysis "Unit Testing Randomness": http://wiki.c2.com/?UnitTestingRandomness
//
//	Kolmogorov-Smirnov test for distributions.
// 		* Fx - True distribution function
// 		* Fn - Empirical (sampled) distribution function (aka function being tested)
// 		* K_plus  - Measures the greatest amount of deviation when Fn is greater than Fx
// 		* K_minus - Measures the maximum deviation when Fn is less than F.
// 		* n_total_samples - Total n samples to collect from Fn
//  Main idea: Check how \textit{different} is Fn from Fx, a critical paramenter is n.
//  		   Large values of n are good to reject the hypothesis that Fn ~= Fx when in reality
//  		   Fn ~= Gx, Gx some distribution other than Fx. However, large n avareges out local
//  		   nonrandom behavior.
//  		   Knuth proposes to use n sufficienlty large (n=1000) and collect r {k_plus, k_minus}
//  		   samples to test if Kn (the empirical function that produces k_plus/k_minus)
//  		   is \textit{similar} to Kx = 1 - e^{-2*x^2}.
//
//

std::function<double(double, double, double)> uniform_function = [](double x, double a, double b) {
  if (x < a)
    return 0.0;
  if (x > b)
    return 1.0;
  return (x - a) / (b - a);
};

std::function<double(double)> Kx_function = [](double x) { return 1.0 - std::exp(-2.0 * std::pow(x, 2)); };

std::tuple<bool, double, double, std::string> kolmogorov_smirnov_test(std::function<double()>& random_function,
                                                                      std::function<double(double)>& Fx)
{
  int n_total_samples = 1'000;
  double failureProbability = 0.001;  // probability of test failing with normal input
  int j;
  std::vector<double> samples;
  samples.reserve(n_total_samples);

  for (j = 0; j != n_total_samples; ++j)
  {
    samples.push_back(random_function());
  }
  std::sort(samples.begin(), samples.end());

  double CDF;
  double temp;
  int j_minus = 0, j_plus = 0;
  double K_plus = std::numeric_limits<double>::min();
  double K_minus = std::numeric_limits<double>::min();
  for (j = 0; j != n_total_samples; ++j)
  {
    CDF = Fx(samples[j]);
    temp = (j + 1.0) / n_total_samples - CDF;
    if (K_plus < temp)
    {
      K_plus = temp;
      j_plus = j;
    }
    temp = CDF - (j + 0.0) / n_total_samples;
    if (K_minus < temp)
    {
      K_minus = temp;
      j_minus = j;
    }
  }
  double sqrtNumReps = std::sqrt(n_total_samples);
  K_plus *= sqrtNumReps;
  K_minus *= sqrtNumReps;
  // We divide the failure probability by four because we have four tests:
  // left and right tests for K+ and K-.
  double p_low = 0.25 * failureProbability;
  double p_high = 1.0 - 0.25 * failureProbability;
  double cutoff_low = std::sqrt(0.5 * std::log(1.0 / (1.0 - p_low))) - 1.0 / (6.0 * sqrtNumReps);
  double cutoff_high = std::sqrt(0.5 * std::log(1.0 / (1.0 - p_high))) - 1.0 / (6.0 * sqrtNumReps);

  std::stringstream ss;
  ss << "\nTesting the random number distribution" << std::endl;
  ss << "Using the Kolmogorov-Smirnov (KS) test." << std::endl;
  ss << "K+ statistic: " << K_plus << std::endl;
  ss << "K+ statistic: " << K_minus << std::endl;
  ss << "Acceptable interval: [" << cutoff_low << ", " << cutoff_high << "]" << std::endl;
  ss << "K+ max at " << j_plus << " " << samples[j_plus] << std::endl;
  ss << "K- max at " << j_minus << " " << samples[j_minus] << std::endl;

  return std::make_tuple(cutoff_low <= K_plus && K_plus <= cutoff_high && cutoff_low <= K_minus &&
                             K_minus <= cutoff_high,
                         K_minus, K_plus, ss.str());
}

BOOST_AUTO_TEST_CASE(baseRandomTestUsingStdUniformDistribution)
{
  prx::init_random(std::random_device()());
  std::function<double()> uniform_random_wrapper = []() {
    // return prx::uniform_random(0.0, 1.0);
    return prx::random::uniform_zero_one(prx::global_generator);
  };

  std::function<double(double)> uniform_function_zero_one = std::bind(uniform_function, std::placeholders::_1, 0, 1);

  bool kolmogorov_smirnov_test_output;
  bool k_minus_kolmogorov_smirnov_test_output;
  bool k_plus_kolmogorov_smirnov_test_output;
  std::string failed_msg_uniform;
  std::string failed_msg_k_minus;
  std::string failed_msg_k_plus;

  std::function<double()> k_minus_Fn = [&]() {
    double k_minus;
    std::tie(std::ignore, k_minus, std::ignore, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_zero_one);
    return k_minus;
  };

  std::function<double()> k_plus_Fn = [&]() {
    double k_plus;
    std::tie(std::ignore, std::ignore, k_plus, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_zero_one);
    return k_plus;
  };

  std::tie(kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_uniform) =
      kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_zero_one);
  std::tie(k_minus_kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_k_minus) =
      kolmogorov_smirnov_test(k_minus_Fn, Kx_function);
  std::tie(k_plus_kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_k_plus) =
      kolmogorov_smirnov_test(k_plus_Fn, Kx_function);

  BOOST_CHECK_MESSAGE(kolmogorov_smirnov_test_output, "Kolmogorov-Smirnov test failed:\n" << failed_msg_uniform);
  BOOST_CHECK_MESSAGE(k_minus_kolmogorov_smirnov_test_output,
                      "K^- Kolmogorov-Smirnov test failed!" << failed_msg_k_minus);
  BOOST_CHECK_MESSAGE(k_plus_kolmogorov_smirnov_test_output,
                      "K^+ Kolmogorov-Smirnov test failed!" << failed_msg_k_plus);
}

BOOST_AUTO_TEST_CASE(ksTestPrxUniformRandom)
{
  prx::init_random(std::random_device()());
  std::function<double()> uniform_random_wrapper = []() {
    // return prx::uniform_random(0.0, 1.0);
    return prx::uniform_random();
  };
  std::function<double(double)> uniform_function_0_1 = std::bind(uniform_function, std::placeholders::_1, 0, 1);

  bool kolmogorov_smirnov_test_output;
  bool k_minus_kolmogorov_smirnov_test_output;
  bool k_plus_kolmogorov_smirnov_test_output;
  std::string failed_msg_uniform;
  std::string failed_msg_k_minus;
  std::string failed_msg_k_plus;

  std::function<double()> k_minus_Fn = [&]() {
    double k_minus;
    std::tie(std::ignore, k_minus, std::ignore, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_0_1);
    return k_minus;
  };

  std::function<double()> k_plus_Fn = [&]() {
    double k_plus;
    std::tie(std::ignore, std::ignore, k_plus, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_0_1);
    return k_plus;
  };

  std::tie(kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_uniform) =
      kolmogorov_smirnov_test(uniform_random_wrapper, uniform_function_0_1);
  std::tie(k_minus_kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_k_minus) =
      kolmogorov_smirnov_test(k_minus_Fn, Kx_function);
  std::tie(k_plus_kolmogorov_smirnov_test_output, std::ignore, std::ignore, failed_msg_k_plus) =
      kolmogorov_smirnov_test(k_plus_Fn, Kx_function);

  BOOST_CHECK_MESSAGE(kolmogorov_smirnov_test_output, "Kolmogorov-Smirnov test failed!" << failed_msg_uniform);
  BOOST_CHECK_MESSAGE(k_minus_kolmogorov_smirnov_test_output,
                      "K^- Kolmogorov-Smirnov test failed!" << failed_msg_k_minus);
  BOOST_CHECK_MESSAGE(k_plus_kolmogorov_smirnov_test_output,
                      "K^+ Kolmogorov-Smirnov test failed!" << failed_msg_k_plus);
}

BOOST_AUTO_TEST_CASE(ksTestPrxUniformRandomCustomRanges)
{
  prx::init_random(std::random_device()());
  std::function<double()> uniform_random_wrapper_0_100 = []() { return prx::uniform_random(0, 100); };
  std::function<double()> uniform_random_wrapper_m100_100 = []() { return prx::uniform_random(-100, 100); };
  std::function<double()> uniform_random_wrapper_m100_0 = []() { return prx::uniform_random(-100, 0); };

  std::function<double(double)> uniform_function_0_100 = std::bind(uniform_function, std::placeholders::_1, 0, 100);
  std::function<double(double)> uniform_function_m100_100 =
      std::bind(uniform_function, std::placeholders::_1, -100, 100);
  std::function<double(double)> uniform_function_m100_0 = std::bind(uniform_function, std::placeholders::_1, -100, 0);

  std::function<double()> k_minus_Fn_0_100 = [&]() {
    double k_minus;
    std::tie(std::ignore, k_minus, std::ignore, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_0_100, uniform_function_0_100);
    return k_minus;
  };

  std::function<double()> k_plus_Fn_0_100 = [&]() {
    double k_plus;
    std::tie(std::ignore, std::ignore, k_plus, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_0_100, uniform_function_0_100);
    return k_plus;
  };

  std::function<double()> k_minus_Fn_m100_100 = [&]() {
    double k_minus;
    std::tie(std::ignore, k_minus, std::ignore, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_m100_100, uniform_function_m100_100);
    return k_minus;
  };

  std::function<double()> k_plus_Fn_m100_100 = [&]() {
    double k_plus;
    std::tie(std::ignore, std::ignore, k_plus, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_m100_100, uniform_function_m100_100);
    return k_plus;
  };

  std::function<double()> k_minus_Fn_m100_0 = [&]() {
    double k_minus;
    std::tie(std::ignore, k_minus, std::ignore, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_m100_0, uniform_function_m100_0);
    return k_minus;
  };

  std::function<double()> k_plus_Fn_m100_0 = [&]() {
    double k_plus;
    std::tie(std::ignore, std::ignore, k_plus, std::ignore) =
        kolmogorov_smirnov_test(uniform_random_wrapper_m100_0, uniform_function_m100_0);
    return k_plus;
  };

  bool kolmogorov_smirnov_test_output_0_100, k_minus_kolmogorov_smirnov_test_output_0_100,
      k_plus_kolmogorov_smirnov_test_output_0_100;
  bool kolmogorov_smirnov_test_output_m100_100, k_minus_kolmogorov_smirnov_test_output_m100_100,
      k_plus_kolmogorov_smirnov_test_output_m100_100;
  bool kolmogorov_smirnov_test_output_m100_0, k_minus_kolmogorov_smirnov_test_output_m100_0,
      k_plus_kolmogorov_smirnov_test_output_m100_0;

  std::string failed_msg_uniform_0_100, failed_msg_k_minus_0_100, failed_msg_k_plus_0_100;
  std::string failed_msg_uniform_m100_100, failed_msg_k_minus_m100_100, failed_msg_k_plus_m100_100;
  std::string failed_msg_uniform_m100_0, failed_msg_k_minus_m100_0, failed_msg_k_plus_m100_0;

  std::tie(kolmogorov_smirnov_test_output_0_100, std::ignore, std::ignore, failed_msg_uniform_0_100) =
      kolmogorov_smirnov_test(uniform_random_wrapper_0_100, uniform_function_0_100);
  std::tie(kolmogorov_smirnov_test_output_m100_100, std::ignore, std::ignore, failed_msg_uniform_m100_100) =
      kolmogorov_smirnov_test(uniform_random_wrapper_m100_100, uniform_function_m100_100);
  std::tie(kolmogorov_smirnov_test_output_m100_0, std::ignore, std::ignore, failed_msg_uniform_m100_0) =
      kolmogorov_smirnov_test(uniform_random_wrapper_m100_0, uniform_function_m100_0);

  std::tie(k_minus_kolmogorov_smirnov_test_output_0_100, std::ignore, std::ignore, failed_msg_k_minus_0_100) =
      kolmogorov_smirnov_test(k_minus_Fn_0_100, Kx_function);
  std::tie(k_minus_kolmogorov_smirnov_test_output_m100_100, std::ignore, std::ignore, failed_msg_k_minus_m100_100) =
      kolmogorov_smirnov_test(k_minus_Fn_m100_100, Kx_function);
  std::tie(k_minus_kolmogorov_smirnov_test_output_m100_0, std::ignore, std::ignore, failed_msg_k_minus_m100_0) =
      kolmogorov_smirnov_test(k_minus_Fn_m100_0, Kx_function);

  std::tie(k_plus_kolmogorov_smirnov_test_output_0_100, std::ignore, std::ignore, failed_msg_k_plus_0_100) =
      kolmogorov_smirnov_test(k_plus_Fn_0_100, Kx_function);
  std::tie(k_plus_kolmogorov_smirnov_test_output_m100_100, std::ignore, std::ignore, failed_msg_k_plus_m100_100) =
      kolmogorov_smirnov_test(k_plus_Fn_m100_100, Kx_function);
  std::tie(k_plus_kolmogorov_smirnov_test_output_m100_0, std::ignore, std::ignore, failed_msg_k_plus_m100_0) =
      kolmogorov_smirnov_test(k_plus_Fn_m100_0, Kx_function);

  BOOST_CHECK_MESSAGE(kolmogorov_smirnov_test_output_0_100,
                      "Kolmogorov-Smirnov test _1 failed!" << failed_msg_uniform_0_100);
  BOOST_CHECK_MESSAGE(kolmogorov_smirnov_test_output_m100_100,
                      "Kolmogorov-Smirnov test _2 failed!" << failed_msg_uniform_m100_100);
  BOOST_CHECK_MESSAGE(kolmogorov_smirnov_test_output_m100_0,
                      "Kolmogorov-Smirnov test _3 failed!" << failed_msg_uniform_m100_0);

  BOOST_CHECK_MESSAGE(k_minus_kolmogorov_smirnov_test_output_0_100,
                      "K^- Kolmogorov-Smirnov test failed!" << failed_msg_k_minus_0_100);
  BOOST_CHECK_MESSAGE(k_minus_kolmogorov_smirnov_test_output_m100_100,
                      "K^- Kolmogorov-Smirnov test failed!" << failed_msg_k_minus_m100_100);
  BOOST_CHECK_MESSAGE(k_minus_kolmogorov_smirnov_test_output_m100_0,
                      "K^- Kolmogorov-Smirnov test failed!" << failed_msg_k_minus_m100_0);

  BOOST_CHECK_MESSAGE(k_plus_kolmogorov_smirnov_test_output_0_100,
                      "K^+ Kolmogorov-Smirnov test failed!" << failed_msg_k_plus_0_100);
  BOOST_CHECK_MESSAGE(k_plus_kolmogorov_smirnov_test_output_m100_100,
                      "K^+ Kolmogorov-Smirnov test failed!" << failed_msg_k_plus_m100_100);
  BOOST_CHECK_MESSAGE(k_plus_kolmogorov_smirnov_test_output_m100_0,
                      "K^+ Kolmogorov-Smirnov test failed!" << failed_msg_k_plus_m100_0);
}

BOOST_AUTO_TEST_CASE(uniform_int_random_within_bounds)
{
  const std::vector<int> lower_bound{ 0, -100, 50, -500 };
  const std::vector<int> upper_bound{ 10, 100, 200, -100 };
  for (int i = 0; i < 1000; ++i)
  {
    const int val = prx::uniform_int_random(lower_bound[0], upper_bound[0]);
    BOOST_CHECK_MESSAGE(lower_bound[0] <= val && val <= upper_bound[0],
                        "prx::uniform_int_random out of bounds " << val);
  }
  for (int i = 0; i < 1000; ++i)
  {
    const int val = prx::uniform_int_random(lower_bound[1], upper_bound[1]);
    BOOST_CHECK_MESSAGE(lower_bound[1] <= val && val <= upper_bound[1],
                        "prx::uniform_int_random out of bounds " << val);
  }
  for (int i = 0; i < 1000; ++i)
  {
    const int val = prx::uniform_int_random(lower_bound[2], upper_bound[2]);
    BOOST_CHECK_MESSAGE(lower_bound[2] <= val && val <= upper_bound[2],
                        "prx::uniform_int_random out of bounds " << val);
  }
  for (int i = 0; i < 1000; ++i)
  {
    const int val = prx::uniform_int_random(lower_bound[3], upper_bound[3]);
    BOOST_CHECK_MESSAGE(lower_bound[3] <= val && val <= upper_bound[3],
                        "prx::uniform_int_random out of bounds " << val);
  }
}