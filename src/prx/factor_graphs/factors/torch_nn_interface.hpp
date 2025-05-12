#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/general/torch_eigen_bridge.hpp"

namespace prx
{
namespace fg
{
template <typename OutType, typename... InTypes>
class torch_nn_interface_t
{
  using Derived = torch_nn_interface_t<OutType, InTypes...>;

  static constexpr Eigen::Index DimOut{ gtsam::traits<OutType>::dimension };
  static constexpr std::size_t NumTypes{ sizeof...(InTypes) };

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  using Error = Eigen::Vector<double, DimX>;
  template <typename Input>
  using Partial = std::function<Error(const Input&)>;
  template <typename Input>
  using FirstOrderDerivative = prx::math::first_order_derivative_t<Partial<Input>, Input, 4>;
  using DifferenceFunction = std::function<X(const X&, const X&)>;

  using Module = torch::jit::script::Module;
  using ModulePtr = std::shared_ptr<Module>;

  inline static DifferenceFunction DefaultDiff = [](const X& a, const X& b) { return a - b; };

  torch_nn_interface_t() = delete;
  torch_nn_interface_t(const torch_nn_interface_t& other) = delete;

public:
  torch_nn_interface_t(ModulePtr module_ptr)
    : Base(), _module_ptr(module_ptr), _in_dim(dimension<InTypes>()), _tensor_output(torch::zeros({ DimOut }))
  {
    _inputs.push_back(torch::zeros({ DimInput }));
  }

  ~torch_nn_interface_t() override
  {
  }

  // template <class... B>
  template <typename... Tp, std::enable_if_t<(sizeof...(Tp) == 0), bool> = true>
  static Eigen::Index dimension()
  {
    return 0;
  }
  template <typename Car, typename... Cdr, std::enable_if_t<(sizeof...(Cdr) >= 1), bool> = true>
  static Eigen::Index dimension()
  {
    return gtsam::traits<Car>::dimension + dimension<Cdr>();
  }

  static ModulePtr load_model(const std::string filename)
  {
    ModulePtr module;
    try
    {
      module = std::make_shared<Module>(torch::jit::load(filename, torch::kCPU));
    }
    catch (const c10::Error& e)
    {
      std::cout << e.what() << "\n";
      prx_throw("[torch_nn_interface_t::load_model] - Error loading the model\n");
    }
    return module;
  }

  OutType operator()(const InTypes&... types, OptionalMatrix<ValueTypes>... H)
  {
    _idx = 0;
    eval(types...);
    prx::copy(_output, _tensor_output);
    return _output;
  }

protected:
  template <typename Car, typename... Cdr, std::enable_if_t<(sizeof...(Cdr) >= 1), bool> = true>
  OutType eval(const Car& car, const Cdr&... cdr)
  {
    static constexpr Eigen::Index DimCar{ gtsam::traits<Car>::dimension };
    // static constexpr Eigen::Index RowsComp{ Eigen::MatrixBase<Derived>::RowsAtCompileTime };
    torch::Tensor tensor{ _inputs[0].index({ torch::indexing::Slice(_idx, DimCar) }) };
    prx::copy(tensor, car);
    _idx += DimCar;
    return eval(cdr...);
  }

  template <typename... Cdr, std::enable_if_t<(sizeof...(Cdr) == 0), bool> = true>
  OutType eval(const Cdr&... types)
  {
    _tensor_output = _model->forward(_inputs).toTensor();
  }

  ModulePtr _module_ptr;
  const Eigen::Index _in_dim;
  Eigen::Index _idx;
  torch::jit::script::Module _module;

  OutType _output;
  at::Tensor _tensor_output;
  std::vector<torch::jit::IValue> _inputs;
};

}  // namespace fg
}  // namespace prx