/*
 * took the structure from PointXYZRGBConfidenceRatio.hpp of the
 * bfe_elevation_mapping package
 *   and adapted it to the SdfFlowPoint structure.
 *
 *   This file defines a point structure for representing SDF flow data in PCL.
 *   It includes fields for position, normal vector, discrete SDF flow value,
 *   angular velocity, and linear velocity.
 *
 *   The structure is registered with PCL to enable its use in point clouds.
 */

#define PCL_NO_PRECOMPILE
#pragma once

#include <pcl/pcl_base.h>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"

namespace pcl {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct _SdfFlowPoint {
  PCL_ADD_POINT4D;                 // preferred way of adding a XYZ+padding
  float nx, ny, nz;                // Normal vector components
  float dsdt;                      // Discrete SDF flow value (D(x,t)/dt)
  float wx, wy, wz;                // Angular velocity components
  float vx, vy, vz;                // Linear velocity components
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment
#pragma GCC diagnostic pop

struct SdfFlowPoint : public _SdfFlowPoint {
  inline explicit SdfFlowPoint(const _SdfFlowPoint& p) : _SdfFlowPoint() {
    // XZY
    x = p.x;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = p.y;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = p.z;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    nx = p.nx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    ny = p.ny;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    nz = p.nz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    dsdt = p.dsdt;  // NOLINT(cppcoreguidelines-pro-type-union-access)
    wx = p.wx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    wy = p.wy;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    wz = p.wz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vx = p.vx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vy = p.vy;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vz = p.vz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  inline explicit SdfFlowPoint(float _dsdt = 0.f)
      : SdfFlowPoint(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, _dsdt) {}

  inline SdfFlowPoint(float _x, float _y, float _z, float _nx = 0.f,
                      float _ny = 0.f, float _nz = 0.f, float _dsdt = 0.f,
                      float _wx = 0.f, float _wy = 0.f, float _wz = 0.f,
                      float _vx = 0.f, float _vy = 0.f, float _vz = 0.f)
      : _SdfFlowPoint() {
    x = _x;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = _y;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = _z;        // NOLINT(cppcoreguidelines-pro-type-union-access)
    nx = _nx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    ny = _ny;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    nz = _nz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    dsdt = _dsdt;  // NOLINT(cppcoreguidelines-pro-type-union-access)
    wx = _wx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    wy = _wy;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    wz = _wz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vx = _vx;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vy = _vy;      // NOLINT(cppcoreguidelines-pro-type-union-access)
    vz = _vz;      // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  friend std::ostream& operator<<(std::ostream& os, const SdfFlowPoint& point) {
    os << "SdfFlowPoint(" << point.x << ", " << point.y << ", " << point.z
       << ", " << point.nx << ", " << point.ny << ", " << point.nz << ", "
       << point.dsdt << ", " << point.wx << ", " << point.wy << ", " << point.wz
       << ", " << point.vx << ", " << point.vy << ", " << point.vz << ")";
    return os;
  }
};

PCL_EXPORTS std::ostream& operator<<(std::ostream& os,
                                     const SdfFlowPoint& point);

}  // namespace pcl

namespace vdbfusion {
using SdfFlowPoint = pcl::SdfFlowPoint;  // Alias for easier usage
using SdfFlowPointCloud =
    pcl::PointCloud<SdfFlowPoint>;  // Alias for point cloud type
using SdfFlowPointCloudPtr =
    std::shared_ptr<SdfFlowPointCloud>;  // Pointer to point cloud
}  // namespace vdbfusion

// Register the point structure with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::_SdfFlowPoint,  // NOLINT(cppcoreguidelines-pro-type-union-access)
    (float, x, x)        // NOLINT
    (float, y, y)        // NOLINT
    (float, z, z)        // NOLINT
    (float, nx, nx)      // NOLINT
    (float, ny, ny)      // NOLINT
    (float, nz, nz)      // NOLINT
    (float, dsdt, dsdt)  // NOLINT
    (float, wx, wx)      // NOLINT
    (float, wy, wy)      // NOLINT
    (float, wz, wz)      // NOLINT
    (float, vx, vx)      // NOLINT
    (float, vy, vy)      // NOLINT
    (float, vz, vz))     // NOLINT

POINT_CLOUD_REGISTER_POINT_WRAPPER(
    pcl::SdfFlowPoint,
    pcl::_SdfFlowPoint)  // NOLINT(cppcoreguidelines-pro-type-union-access)

// Explicit template instantiation for PCL algorithms
template class pcl::PointCloud<pcl::SdfFlowPoint>;
template class pcl::PCLBase<pcl::SdfFlowPoint>;
template class pcl::KdTreeFLANN<pcl::SdfFlowPoint>;
// template class pcl::getMinMax3D<pcl::SdfFlowPoint>;