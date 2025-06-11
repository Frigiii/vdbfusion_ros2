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
  std::shared_ptr<openvdb::FloatGrid> inline CreateOneShotTSDF(
      const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
      const std::function<float(float)>& variance_function) {
    std::shared_ptr<openvdb::FloatGrid> os_tsdf =
        openvdb::FloatGrid::create(sdf_trunc_);
    os_tsdf->setName("One-shot TSDF");
    os_tsdf->setTransform(
        openvdb::math::Transform::createLinearTransform(voxel_size_));
    os_tsdf->setGridClass(openvdb::GRID_LEVEL_SET);
    Integrate(points, origin, variance_function, os_tsdf, nullptr,
              voxel_size_ / 2.0f);
    return os_tsdf;
  }

  std::shared_ptr<openvdb::FloatGrid> GetSparseFlowField(
      std::shared_ptr<openvdb::FloatGrid> os_tsdf);

  void ApplyFlowField(std::shared_ptr<openvdb::FloatGrid> flow_field) {
    // Apply the flow field to the current tsdf_ grid
    using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;
    AccessorRW tsdf_acc = AccessorRW(tsdf_->tree());
    AccessorRW flow_acc = AccessorRW(flow_field->tree());
    for (auto iter = flow_field->cbeginValueOn(); iter; ++iter) {
      const openvdb::Coord& voxel = iter.getCoord();
      if (tsdf_acc.isValueOn(voxel)) {
        // Update the tsdf_ value by adding the flow field value
        const float tsdf_value = tsdf_acc.getValue(voxel);
        const float flow_value = flow_acc.getValue(voxel);
        tsdf_acc.setValue(voxel, tsdf_value + flow_value);
      }
    }
  }

 public:
  SdfFlowPointCloudPtr latest_flow_field_ =
      std::make_shared<SdfFlowPointCloud>();  ///< Latest flow field

  /// @brief Returns the latest flow field
  SdfFlowPointCloudPtr getLatestFlowField() const { return latest_flow_field_; }

  /// @brief Sets the latest flow field
  void setLatestFlowField(const SdfFlowPointCloudPtr& flow_field) {
    latest_flow_field_ = flow_field;
  }
};

}  // namespace vdbfusion