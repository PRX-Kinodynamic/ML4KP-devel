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
struct naive_quat_diff_t
{
  gtsam::Rot3 qfix;
  Eigen::Vector4d operator()(const Eigen::Vector4d& qv) const
  {
    const gtsam::Rot3 x0{ qv[0], qv[1], qv[2], qv[3] };
    const gtsam::Rot3 between{ gtsam::traits<gtsam::Rot3>::Between(x0, qfix) };
    return quat_to_vec(between.toQuaternion());
  }

  static Eigen::Vector4d quat_to_vec(const Eigen::Quaterniond& q)
  {
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  }
};

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

int main(int argc, char* argv[])
{
  using Derivative = prx::math::first_order_derivative_t<naive_quat_diff_t, Eigen::Vector4d, 5, -2>;

  const gtsam::Rot3 r_fix(1., 0., 0., 0.);

  Eigen::Matrix3d H_rfix, H_ri;
  Eigen::Matrix4d H_vfix, H_vi;

  std::ofstream ofs_hrfix(prx::out_path + "/h_rfix.txt");
  std::ofstream ofs_hri(prx::out_path + "/h_ri.txt");
  std::ofstream ofs_hvfix(prx::out_path + "/h_vfix.txt");
  std::ofstream ofs_hvi(prx::out_path + "/h_vi.txt");
  std::ofstream ofs_r_error(prx::out_path + "/r_error.txt");
  std::ofstream ofs_v_error(prx::out_path + "/v_error.txt");
  std::ofstream ofs_quat(prx::out_path + "/quat.txt");
  double step{ 0.1 };
  gtsam::Rot3 ri(1, 0, 0, 0);
  naive_quat_diff_t naive;
  naive.qfix = r_fix;
  Derivative derivative(0.001);

  for (double x = -1. * prx::constants::pi; x < 1. * prx::constants::pi; x += step)
  {
    for (double y = -1. * prx::constants::pi; y < 1. * prx::constants::pi; y += step)
    {
      for (double z = -1. * prx::constants::pi; z < 1. * prx::constants::pi; z += step)
      {
        const Eigen::Vector3d delta{ step * std::sin(x), step * std::sin(y), step * std::sin(z) };

        const Eigen::Quaterniond q{ ri.toQuaternion() };
        const Eigen::Vector4d vi(q.w(), q.x(), q.y(), q.z());
        const Eigen::Vector3d rot_error{ compute_error(r_fix, ri, H_rfix, H_ri) };
        const Eigen::Vector4d vec_error{ naive(vi) };
        H_vi = derivative(vi);

        matrix_to_file(ofs_hrfix, H_rfix);
        matrix_to_file(ofs_hri, H_ri);
        // matrix_to_file(ofs_hvfix, H_vfix);
        matrix_to_file(ofs_hvi, H_vi);
        ofs_r_error << rot_error.transpose() << "\n";
        ofs_v_error << vec_error.transpose() << "\n";
        ofs_quat << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        ofs_quat << "\n";
        // ofs_quat << delta[0] << " " << delta[1] << " " << delta[2] << "\n";

        ri = gtsam::traits<gtsam::Rot3>::Compose(ri, gtsam::traits<gtsam::Rot3>::Expmap(delta));
      }
    }
  }

  ofs_hrfix.close();
  ofs_hri.close();
  ofs_hvfix.close();
  ofs_hvi.close();
  ofs_r_error.close();
  ofs_v_error.close();

  return 0;
}