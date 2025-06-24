// MIT License
//
// # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <vector>

#include "vdbfusion/VolumeExtractor.h"

namespace vdbfusion {
void VolumeExtractor::updateVolume() {
  VDBFUSION_VOLUMEEXTRACTOR_ASSERT(tsdf_ && boundary_lower_ && boundary_upper_);

  // Create a new extract volume if it does not exist. Same properties as tsdf_
  if (!extract_volume_lower_) {
    extract_volume_lower_ = openvdb::FloatGrid::create(tsdf_->background());
    extract_volume_lower_->setTransform(tsdf_->transformPtr());
  }
  if (!extract_volume_upper_) {
    extract_volume_upper_ = openvdb::FloatGrid::create(tsdf_->background());
    extract_volume_upper_->setTransform(tsdf_->transformPtr());
  }

  // Clear the existing volume
  extract_volume_lower_->clear();
  extract_volume_upper_->clear();

  // Create the intersection of the volume and boundary and calculate the volume
  // value
  size_t count_lower = 0;
  size_t count_upper = 0;
  auto xform = tsdf_->transform();
  const float voxel_size = xform.voxelSize()[0];

  auto tsdf_acc = tsdf_->getAccessor();
  auto boundary_lower_acc = boundary_lower_->getAccessor();
  auto boundary_upper_acc = boundary_upper_->getAccessor();
  auto extract_volume_lower_acc = extract_volume_lower_->getAccessor();
  auto extract_volume_upper_acc = extract_volume_upper_->getAccessor();

  for (auto iter = boundary_lower_->cbeginValueOn(); iter.test(); ++iter) {
    // Get the coordinate of the current voxel
    const openvdb::Coord& coord = iter.getCoord();

    if (!tsdf_acc.isValueOn(coord)) continue;

    const float tsdf_value = tsdf_acc.getValue(coord);

    // Check if the coordinate is active in both grids
    if (boundary_lower_acc.isValueOn(coord)) {
      // Get the values from both grids
      const float boundary_value = boundary_lower_acc.getValue(coord);
      const float extract_value = std::max(tsdf_value, boundary_value);

      if (extract_value < iso_level_) ++count_lower;
      extract_volume_lower_acc.setValue(coord, extract_value);
    }

    if (boundary_upper_acc.isValueOn(coord)) {
      // Get the values from both grids
      const float boundary_value = boundary_upper_acc.getValue(coord);
      const float extract_value = std::max(tsdf_value, boundary_value);

      if (extract_value < iso_level_) ++count_upper;
      extract_volume_upper_acc.setValue(coord, extract_value);
    }
  }

  for (auto iter = boundary_upper_->cbeginValueOn(); iter.test(); ++iter) {
    // Get the coordinate of the current voxel
    const openvdb::Coord& coord = iter.getCoord();

    // Skip if the coordinate is already processed
    if (!extract_volume_upper_acc.isValueOn(coord) &&
        tsdf_acc.isValueOn(coord) && boundary_upper_acc.isValueOn(coord)) {
      const float tsdf_value = tsdf_acc.getValue(coord);
      const float boundary_value = boundary_upper_acc.getValue(coord);
      const float extract_value = std::max(tsdf_value, boundary_value);
      if (extract_value < iso_level_) ++count_upper;
      extract_volume_upper_acc.setValue(coord, extract_value);
    }
  }

  volume_value_lower_ = static_cast<float>(count_lower) * voxel_size *
                        voxel_size * voxel_size;  // Volume value
  volume_value_upper_ = static_cast<float>(count_upper) * voxel_size *
                        voxel_size * voxel_size;  // Volume value

  // The volume_value_ should be interpolated properly. This is just a very
  // stupid interpolation based on the lower volume.
  const float volume_lower_max = 1.041f;
  const float lower_fill_ratio =
      volume_value_lower_ / volume_lower_max;  // Ratio of the lower volume
  const float a = 0.8f;
  const float b = 1.0f;
  if (lower_fill_ratio < a) {
    volume_value_ = volume_value_lower_;
  } else if (lower_fill_ratio > b) {
    volume_value_ = volume_value_upper_;
  } else {
    // Interpolate
    volume_value_ =
        volume_value_lower_ + (volume_value_upper_ - volume_value_lower_) *
                                  (lower_fill_ratio - a) / (b - a);
  }
}

void VolumeExtractor::loadBoundaryMesh(const std::string& boundary_mesh,
                                       GridPtr& boundary, std::string name) {
  VDBFUSION_VOLUMEEXTRACTOR_ASSERT(!boundary_mesh.empty() &&
                                   std::filesystem::exists(boundary_mesh));
  // Load the boundary mesh using tinyobjloader. Convert the mesh to a VDB SDF
  // and store it in the boundary member variable

  std::vector<openvdb::Vec3s> boundary_points;
  std::vector<openvdb::Vec3I> boundary_triangles;
  std::vector<openvdb::Vec4I> boundary_quads;

  ///////////////////////////////////////
  // Load the mesh using tinyobjloader //
  ///////////////////////////////////////

  tinyobj::ObjReaderConfig reader_config;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(boundary_mesh, reader_config)) {
    throw std::runtime_error("Failed to parse boundary mesh: " +
                             reader.Error());
  }

  if (!reader.Warning().empty()) {
    std::cerr << "Warning while loading boundary mesh: " << reader.Warning()
              << std::endl;
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  std::cout << "Loaded boundary mesh: " << boundary_mesh << std::endl;
  std::cout << "Number of vertices: " << attrib.vertices.size() / 3
            << std::endl;
  std::cout << "Number of shapes: " << shapes.size() << std::endl;
  std::cout << "Number of materials: " << materials.size() << std::endl;

  // Fill points
  for (size_t v = 0; v < attrib.vertices.size() / 3; ++v) {
    boundary_points.emplace_back(attrib.vertices[3 * v],
                                 attrib.vertices[3 * v + 1],
                                 attrib.vertices[3 * v + 2]);
  }

  // Fill triangles and quads
  for (const auto& shape : shapes) {
    size_t index_offset = 0;
    for (const auto& num_vertices : shape.mesh.num_face_vertices) {
      if (num_vertices < 3) {
        continue;  // Skip non-triangular faces
      }
      std::vector<int> face_indices;
      for (size_t v = 0; v < num_vertices; ++v) {
        int idx = shape.mesh.indices[index_offset + v].vertex_index;
        face_indices.push_back(idx);
      }
      index_offset += num_vertices;
      if (num_vertices == 3) {
        // Triangle
        if (face_indices.size() == 3) {
          boundary_triangles.emplace_back(openvdb::Vec3I(
              face_indices[0], face_indices[1], face_indices[2]));
        }
      } else if (num_vertices == 4) {
        // Quad
        if (face_indices.size() == 4) {
          boundary_quads.emplace_back(
              openvdb::Vec4I(face_indices[0], face_indices[1], face_indices[2],
                             face_indices[3]));
        }
      }
    }
  }

  ///////////////////////////////////
  // Convert the mesh to a VDB SDF //
  ///////////////////////////////////

  auto xform = tsdf_->transform();

  // Typical values for exBandWidth and inBandWidth are 3.0 to 5.0 times the
  // voxel size. To have the SDF filled on the inside and empty on the
  // outside, set inBandWidth large enough to cover the interior. Example:
  // const float voxel_size = xform.voxelSize()[0];
  // float exBandWidth = 3.0f * voxel_size;  // exterior narrow band width
  // float inBandWidth = 500.0f * voxel_size;  // interior narrow band width$

  std::cout << "Converting boundary mesh to VDB SDF..." << std::endl;
  std::cout << "Number of boundary points: " << boundary_points.size()
            << std::endl;
  std::cout << "Number of boundary triangles: " << boundary_triangles.size()
            << std::endl;
  std::cout << "Number of boundary quads: " << boundary_quads.size()
            << std::endl;

  boundary = openvdb::tools::meshToSignedDistanceField<GridType>(
      xform, boundary_points, boundary_triangles, boundary_quads, 3, 500);

  boundary->setName(name);
  boundary->setTransform(tsdf_->transformPtr());
}
}  // namespace vdbfusion