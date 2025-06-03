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
#include <tiny_obj_loader.h>

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <vector>

#include "vdbfusion/VolumeExtractor.h"

namespace vdbfusion {
void VolumeExtractor::updateVolume() {
  VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_ && tsdf_ && boundary_);

  // Clear the existing volume
  extract_volume_->tree().clear();

  // Create the intersection of the volume and boundary and calculate the volume
  // value
  //   TODO
}

void VolumeExtractor::loadBoundaryMesh(const std::string& boundary_mesh) {
  VDBFUSION_VOLUMEEXTRACTOR_ASSERT(!boundary_mesh.empty() &&
                                   std::filesystem::exists(boundary_mesh));
  // Load the boundary mesh using tinyobjloader. Convert the mesh to a VDB SDF
  // and store it in the boundary_ member variable

  std::vector<openvdb::Vec3s> boundary_points;
  std::vector<openvdb::Vec3I> boundary_triangles;
  std::vector<openvdb::Vec4I> boundary_quads;

  ///////////////////////////////////////
  // Load the mesh using tinyobjloader //
  ///////////////////////////////////////

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        boundary_mesh.c_str())) {
    throw std::runtime_error("Failed to load boundary mesh: " + warn + err);
  }

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
  // voxel size. To have the SDF filled on the inside and empty on the outside,
  // set inBandWidth large enough to cover the interior. Example:
  //   const float voxel_size = xform.voxelSize()[0];
  //   float exBandWidth = 3.0f * voxel_size;  // exterior narrow band width
  //   float inBandWidth = 5.0f * voxel_size;  // interior narrow band width

  boundary_ = openvdb::tools::meshToLevelSet<GridType>(
      xform, boundary_points, boundary_triangles, boundary_quads);
}
}  // namespace vdbfusion