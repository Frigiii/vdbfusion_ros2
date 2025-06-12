#pragma once

#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <functional>
#include <tuple>

#include "vdbfusion/SdfFlowPoint.hpp"
#include "vdbfusion/VDBVolume.h"

namespace vdbfusion {
class DiscreteSDFFlow : public VDBVolume {
 public:
  DiscreteSDFFlow(float voxel_size, float sdf_trunc, bool space_carving = false,
                  float min_var = 0.0f, float var_punish = 0.0f,
                  float tsdf_punish = 0.0f)
      : VDBVolume(voxel_size, sdf_trunc, space_carving, min_var, var_punish,
                  tsdf_punish) {}

  ~DiscreteSDFFlow() = default;

  /// @brief OVERRIDE FUNCTION : Integrates a new (globally aligned) PointCloud
  /// into the current tsdf_ volume while considering a discrete flow field.
  void Integrate(const std::vector<Eigen::Vector3d>& points,
                 const Eigen::Vector3d& origin,
                 const std::function<float(float)>& variance_function);

  /// @brief Integrates a new (globally aligned) PointCloud
  /// into a given tsdf and optionally into a given vars grid as well as the
  /// integration distance. With given integration distance, the space carving
  /// is disabled.
  void Integrate(const std::vector<Eigen::Vector3d>& points,
                 const Eigen::Vector3d& origin,
                 const std::function<float(float)>& variance_function,
                 std::shared_ptr<openvdb::FloatGrid> tsdf,
                 std::shared_ptr<openvdb::FloatGrid> vars = nullptr,
                 float integration_distance = -1.0f);

  /// @brief Creates a one-shot TSDF from the given points and origin.
  std::shared_ptr<openvdb::FloatGrid> CreateOneShotTSDF(
      const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
      const std::function<float(float)>& variance_function);

  std::shared_ptr<openvdb::FloatGrid> GetSparseSdfFlow(
      std::shared_ptr<openvdb::FloatGrid> os_tsdf, const float dt);

  void ApplySparseSceneFlow(
      std::shared_ptr<openvdb::FloatGrid> sparse_scene_flow);

  SdfFlowPointCloudPtr EstimateSparseSceneFlow(
      std::shared_ptr<openvdb::FloatGrid> dsdt,
      std::shared_ptr<openvdb::Vec3fGrid> grad_s, float downsample_radius,
      int min_num_nn);

 public:
  SdfFlowPointCloudPtr latest_sparse_scene_flow_ =
      std::make_shared<SdfFlowPointCloud>();  ///< Latest flow field

  /// @brief Returns the latest flow field
  SdfFlowPointCloudPtr getLatestSparseSceneFlow() const {
    return latest_sparse_scene_flow_;
  }

  /// @brief Sets the latest flow field
  void setLatestSparseSceneFlow(const SdfFlowPointCloudPtr& sparse_scene_flow) {
    latest_sparse_scene_flow_ = sparse_scene_flow;
  }
};

}  // namespace vdbfusion