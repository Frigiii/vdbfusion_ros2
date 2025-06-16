/*
 * MIT License
 *
 * # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <functional>
#include <tuple>

#include "vdbfusion/VolumeExtractor.h"

namespace vdbfusion {

class VDBVolume {
 public:
  VDBVolume(float voxel_size, float sdf_trunc, bool space_carving = false,
            float min_var = 100.0f, float var_punish = 0.0f,
            float tsdf_punish = 0.0f);
  ~VDBVolume() = default;

 public:
  /// @brief Integrates a new (globally aligned) PointCloud into the current
  /// tsdf_ volume.
  void Integrate(const std::vector<Eigen::Vector3d>& points,
                 const Eigen::Vector3d& origin,
                 const std::function<float(float)>& variance_function);

  /// @brief Integrates a new (globally aligned) PointCloud into the current
  /// tsdf_ volume.
  void inline Integrate(const std::vector<Eigen::Vector3d>& points,
                        const Eigen::Matrix4d& extrinsics,
                        const std::function<float(float)>& variance_function) {
    const Eigen::Vector3d& origin = extrinsics.block<3, 1>(0, 3);
    Integrate(points, origin, variance_function);
  }

  /// @brief Integrate incoming TSDF grid inside the current volume using the
  /// TSDF equations
  void Integrate(openvdb::FloatGrid::Ptr grid,
                 const std::function<float(float)>& variance_function);

  /// @brief Fuse a new given sdf value at the given voxel location, thread-safe
  void UpdateTSDF(const float& sdf, const openvdb::Coord& voxel,
                  const std::function<float(float)>& variance_function);

  /// @brief Prune TSDF grids, ideal utility to cleanup a D(x) volume before
  /// exporting it
  openvdb::FloatGrid::Ptr Prune(float max_var) const;

  /// @brief Extracts a TriangleMesh as the iso-surface in the actual volume
  [[nodiscard]] std::tuple<std::vector<Eigen::Vector3d>,
                           std::vector<Eigen::Vector3i>>
  ExtractTriangleMesh(bool fill_holes = true, float max_var = 0.5,
                      openvdb::FloatGrid::Ptr tsdf = nullptr,
                      float iso_level = 0.0f) const;

  void PunishNotUpdatedVoxels();

  void initVolumeExtractor(std::string lower_boundary_mesh_path,
                           std::string upper_boundary_mesh_path,
                           float iso_level = 0.0f) {
    volume_extractor_ = VolumeExtractor(tsdf_, iso_level);
    volume_extractor_.loadBoundaryMesh(lower_boundary_mesh_path,
                                       upper_boundary_mesh_path);
  }

  void updateVolumeExtractor() { volume_extractor_.updateVolume(); }

  openvdb::FloatGrid::Ptr getVolumeExtractorVolumeLower() const {
    return volume_extractor_.getExtractorVolumeLower();
  }
  openvdb::FloatGrid::Ptr getVolumeExtractorVolumeUpper() const {
    return volume_extractor_.getExtractorVolumeUpper();
  }

  float getVolumeValue() const { return volume_extractor_.getVolumeValue(); }
  float getVolumeValueLower() const {
    return volume_extractor_.getVolumeValueLower();
  }
  float getVolumeValueUpper() const {
    return volume_extractor_.getVolumeValueUpper();
  }

 public:
  /// OpenVDB Grids modeling the signed distance field and the var grid
  openvdb::FloatGrid::Ptr tsdf_;
  openvdb::FloatGrid::Ptr variance_;
  openvdb::BoolGrid::Ptr updated_;

  VolumeExtractor volume_extractor_;

  /// VDBVolume public properties
  float voxel_size_;
  float sdf_trunc_;
  bool space_carving_;
  float max_var_;
  float iso_level_;
  float min_var_ = 0.0f;  // default value for variance

  float var_punish_;
  float tsdf_punish_;
};

}  // namespace vdbfusion
