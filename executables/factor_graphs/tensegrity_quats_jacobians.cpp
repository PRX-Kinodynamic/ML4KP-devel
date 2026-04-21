#include <gtsam/geometry/Rot3.h>
#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

template <typename Matrix>
void matrix_to_file(std::ofstream& ofs, const Matrix& mat)
{
  for (int i = 0; i < mat.rows(); ++i)
  {
    for (int j = 0; j < mat.cols(); ++j)
    {
      ofs << mat(i, j) << " ";
    }
  }
  ofs << "\n";
}

template <typename Element, typename Matrix>
Eigen::VectorXd compute_error(const Element& x0, const Element& x1, Matrix& err_H_x0, Matrix& err_H_x1)
{
  Matrix b_H_rfix, b_H_ri, err_H_b;

  const Element between{ gtsam::traits<Element>::Between(x0, x1, b_H_rfix, b_H_ri) };
  const Eigen::VectorXd error{ gtsam::traits<Element>::Logmap(between, err_H_b) };
  err_H_x0 = err_H_b * b_H_rfix;
  err_H_x1 = err_H_b * b_H_ri;
  return error;
}

std::vector<std::pair<gtsam::Rot3, gtsam::Rot3>> read_quats(const std::string filename)
{
  using prx::utilities::convert_to;
  using CsvReader = prx::utilities::csv_reader_t;
  CsvReader reader(filename);
  // # t  qw_t  qx_t  qy_t  qz_t  qw_t+1  qx_t+1  qy_t+1  qz_t+1
  // 1  0.888930  -0.296977  0.348723  0.000000  0.888993  -0.296974  0.348565  0.000000
  std::vector<std::pair<gtsam::Rot3, gtsam::Rot3>> quats;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      continue;
    if (line[0][0] == '#')
      continue;
    const double q0w{ convert_to<double>(line[1]) };
    const double q0x{ convert_to<double>(line[2]) };
    const double q0y{ convert_to<double>(line[3]) };
    const double q0z{ convert_to<double>(line[4]) };

    const double q1w{ convert_to<double>(line[5]) };
    const double q1x{ convert_to<double>(line[6]) };
    const double q1y{ convert_to<double>(line[7]) };
    const double q1z{ convert_to<double>(line[8]) };

    const gtsam::Rot3 q0(q0w, q0x, q0y, q0z);
    const gtsam::Rot3 q1(q1w, q1x, q1y, q1z);
    quats.push_back({ q0, q1 });
  }
  return quats;
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params.add_opts(argc, argv);

  const std::string filename{ params["filename"].as<>() };
  std::vector<std::pair<gtsam::Rot3, gtsam::Rot3>> quats{ read_quats(filename) };

  Eigen::Matrix3d Hq0, Hq1;

  std::ofstream ofs_q_error(prx::out_path + "/q01_err.txt");
  std::ofstream ofs_hq0(prx::out_path + "/hq0.txt");
  std::ofstream ofs_hq1(prx::out_path + "/hq1.txt");

  for (auto& q01 : quats)
  {
    auto q0 = q01.first;
    auto q1 = q01.second;
    const Eigen::Vector3d rot_error{ compute_error(q0, q1, Hq0, Hq1) };

    ofs_q_error << q0.toQuaternion().w() << " " << q0.toQuaternion().x() << " " << q0.toQuaternion().y() << " "
                << q0.toQuaternion().z() << " ";
    ofs_q_error << q1.toQuaternion().w() << " " << q1.toQuaternion().x() << " " << q1.toQuaternion().y() << " "
                << q1.toQuaternion().z() << " ";
    ofs_q_error << rot_error.transpose() << "\n";

    matrix_to_file(ofs_hq0, Hq0);
    matrix_to_file(ofs_hq1, Hq1);
  }

  ofs_q_error.close();
  ofs_hq0.close();
  ofs_hq1.close();
  return 0;
}