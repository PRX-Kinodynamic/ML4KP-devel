#include "prx/factor_graphs/utilities/formatter.hpp"

namespace prx
{
namespace fg
{
void formatter_t::save_as_graphviz_file(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                                        const gtsam::Values& values, const gtsam::KeyFormatter& keyFormatter) const
{
  std::ofstream ofs_graph;
  ofs_graph.open(filename.c_str(), std::ios::trunc);

  // write preamble: no size because is not "standard" and some libraries do not recognize it...
  ofs_graph << "graph {\n";

  // Find bounds (imperative)
  gtsam::KeySet keys = graph.keys();
  // Vector2 min = writer.findBounds(values, keys);

  // // Create nodes for each variable in the graph
  // for (Key key : keys)
  // {
  //   auto position = writer.variablePos(values, min, key);
  //   writer.drawVariable(key, keyFormatter, position, &os);
  // }
  // os << "\n";

  // if (writer.mergeSimilarFactors)
  // {
  //   // Remove duplicate factors
  //   std::set<KeyVector> structure;
  //   for (const sharedFactor& factor : factors_)
  //   {
  //     if (factor)
  //     {
  //       KeyVector factorKeys = factor->keys();
  //       std::sort(factorKeys.begin(), factorKeys.end());
  //       structure.insert(factorKeys);
  //     }
  //   }

  //   // Create factors and variable connections
  //   size_t i = 0;
  //   for (const KeyVector& factorKeys : structure)
  //   {
  //     writer.processFactor(i++, factorKeys, keyFormatter, boost::none, &os);
  //   }
  // }
  // else
  // {
  //   // Create factors and variable connections
  //   for (size_t i = 0; i < size(); ++i)
  //   {
  //     const NonlinearFactor::shared_ptr& factor = at(i);
  //     if (factor)
  //     {
  //       const KeyVector& factorKeys = factor->keys();
  //       writer.processFactor(i, factorKeys, keyFormatter, writer.factorPos(min, i), &os);
  //     }
  //   }
  // }

  ofs_graph << "}\n";
  std::flush(ofs_graph);
}
}  // namespace fg
}  // namespace prx