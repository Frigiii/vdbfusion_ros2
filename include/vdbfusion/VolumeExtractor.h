#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#define VDBFUSION_VOLUMEEXTRACTOR_ASSERTS

#ifdef VDBFUSION_VOLUMEEXTRACTOR_ASSERTS
#include <stdexcept>
#define VDBFUSION_VOLUMEEXTRACTOR_ASSERT(condition)                       \
  if (!(static_cast<bool>(condition)))                                    \
    throw std::runtime_error("VolumeExtractor assert failed: " #condition \
                             " at " __FILE__ ":" +                        \
                             std::to_string(__LINE__));
#else
#define VDBFUSION_VOLUMEEXTRACTOR_ASSERT(...)
#endif

namespace vdbfusion {

using GridType = openvdb::FloatGrid;
using GridPtr = GridType::Ptr;

class VolumeExtractor {
 public:
  VolumeExtractor() = default;
  VolumeExtractor(const GridPtr tsdf, float iso_level = 0.0f)
      : tsdf_(tsdf),
        iso_level_(iso_level),
        volume_value_(0.0),
        volume_value_lower_(0.0),
        volume_value_upper_(0.0) {}
  ~VolumeExtractor() = default;

  /// @brief Extracts a TriangleMesh as the iso-surface in the actual volume
  [[nodiscard]] std::tuple<std::vector<Eigen::Vector3d>,
                           std::vector<Eigen::Vector3i>>
  ExtractTriangleMesh(bool fill_holes = true, float max_var = 0.5) const;

  /// @brief Creates the intersection of volume_.tsdf_ and boundary_ and
  /// stores the result in extract_volume_. Additionally updates the
  /// volume_value_
  void updateVolume();

  /// @brief Extracts the volume value from the volume at the iso_level
  float getVolumeValue() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_lower_ && tsdf_ &&
                                     boundary_lower_);
    return volume_value_;
  }
  float getVolumeValueLower() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_lower_ && tsdf_ &&
                                     boundary_lower_);
    return volume_value_lower_;
  }
  float getVolumeValueUpper() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_upper_ && tsdf_ &&
                                     boundary_upper_);
    return volume_value_upper_;
  }

  GridPtr getExtractorVolumeLower() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_lower_);
    return extract_volume_lower_;
  }

  GridPtr getExtractorVolumeUpper() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_upper_);
    return extract_volume_upper_;
  }

  /// @brief Loads a boundary mesh from a file and converts it to sdf into
  /// boundary_
  void loadBoundaryMesh(const std::string& boundary_mesh, GridPtr& boundary,
                        std::string name = "boundary");

  void loadBoundaryMesh(const std::string& lower_boundary_mesh,
                        const std::string& upper_boundary_mesh) {
    loadBoundaryMesh(lower_boundary_mesh, boundary_lower_, "lower_boundary");
    loadBoundaryMesh(upper_boundary_mesh, boundary_upper_, "upper_boundary");
  }

 private:
  //   const VDBVolume& volume_;
  float volume_value_;
  float volume_value_lower_;
  float volume_value_upper_;
  float iso_level_;

  GridPtr tsdf_;
  GridPtr extract_volume_lower_;
  GridPtr extract_volume_upper_;
  GridPtr boundary_lower_;
  GridPtr boundary_upper_;
};

}  // namespace vdbfusion