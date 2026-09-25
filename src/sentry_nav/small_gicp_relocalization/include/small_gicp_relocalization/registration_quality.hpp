#ifndef SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_
#define SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_

#include <cmath>
#include <cstddef>

namespace small_gicp_relocalization
{
// Iteration-limit results may still be usable, but must pass exactly the same
// overlap/fitness gates as formally converged results. NaN must never pass.
inline bool registrationQualityAcceptable(
  bool converged, size_t inliers, size_t source_size, double error,
  double quality_threshold, double min_ratio, double max_fitness)
{
  if (inliers < 50 || source_size == 0 || !std::isfinite(error) || error < 0.0) {
    return false;
  }
  const double fitness = error / static_cast<double>(inliers);
  return (converged || fitness < quality_threshold) &&
         static_cast<double>(inliers) / source_size >= min_ratio && fitness <= max_fitness;
}
// A known-start local search must stay near the supplied prior, even if a
// distant map region has a good numerical score.
inline bool initialCorrectionAcceptable(
  double dx, double dy, double yaw_delta, double max_distance, double max_yaw)
{
  return std::isfinite(dx) && std::isfinite(dy) && std::isfinite(yaw_delta) &&
         std::hypot(dx, dy) <= max_distance &&
         std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta))) <= max_yaw;
}
}  // namespace small_gicp_relocalization
#endif
