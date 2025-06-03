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
      : tsdf_(tsdf), iso_level_(iso_level) {}
  ~VolumeExtractor() = default;

  /// @brief Extracts a TriangleMesh as the iso-surface in the actual volume
  [[nodiscard]] std::tuple<std::vector<Eigen::Vector3d>,
                           std::vector<Eigen::Vector3i>>
  ExtractTriangleMesh(bool fill_holes = true, float min_weight = 0.5) const;

  /// @brief Creates the intersection of volume_.tsdf_ and boundary_ and
  /// stores the result in extract_volume_. Additionally updates the
  /// volume_value_
  void updateVolume();

  /// @brief Extracts the volume value from the volume at the iso_level
  float getVolumeValue() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_ && tsdf_ && boundary_);
    return volume_value_;
  }

  GridPtr getExtractVolume() const {
    VDBFUSION_VOLUMEEXTRACTOR_ASSERT(extract_volume_);
    // return boundary_;
    return extract_volume_;
  }

  /// @brief Loads a boundary mesh from a file and converts it to sdf into
  /// boundary_
  void loadBoundaryMesh(const std::string& boundary_mesh);

 private:
  //   const VDBVolume& volume_;
  float volume_value_;
  float iso_level_;

  GridPtr tsdf_;
  GridPtr extract_volume_;
  GridPtr boundary_;
};

}  // namespace vdbfusion