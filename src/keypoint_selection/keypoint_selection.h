#pragma once
#include <algorithm>
#include <execution>
#include <numeric>
#include <vector>

#include "core/Frame.h"
#include "core/types.h"
#include "core/random.h"
namespace vslam::keypoint
{
template <typename Criterion>
std::vector<Vec2d> select(Frame::ConstShPtr f, int level, Criterion criterion)
{
  std::vector<int> vs(f->height(level));
  for (size_t v = 0; v < f->height(level); v++) {
    vs[v] = v;
  }

  std::vector<Vec2d> keypoints = std::transform_reduce(
    std::execution::par_unseq, vs.begin(), vs.end(), std::vector<Vec2d>(),
    [](auto v0, auto v1) {
      v0.insert(v0.end(), v1.begin(), v1.end());
      return v0;
    },
    [&](int v) {
      std::vector<Vec2d> kps;
      kps.reserve(f->width(level));
      for (size_t u = 0; u < f->width(level); u++) {
        if (criterion(Vec2d(u, v))) {
          kps.push_back(Vec2d(u, v));
        }
      }
      return kps;
    });
  return keypoints;
}
namespace subsampling
{
template <typename KeyPoint, typename Position>
std::vector<KeyPoint> uniform(
  const std::vector<KeyPoint> & keypoints, int height, int width, int nPoints, Position getPosition)
{
  const size_t nNeeded = std::max<size_t>(20, nPoints);
  std::vector<bool> mask(height * width, false);
  std::vector<KeyPoint> subset;
  subset.reserve(keypoints.size());
  if (nNeeded < keypoints.size()) {
    while (subset.size() < nNeeded) {
      auto kp = keypoints[random::U(0, keypoints.size() - 1)];
      const auto & pos = getPosition(kp);
      const size_t idx = pos(1) * width + pos(0);
      if (!mask[idx]) {
        subset.push_back(kp);
        mask[idx] = true;
      }
    }
    return subset;
  }
  return keypoints;
}
}  // namespace subsampling

}  // namespace vslam::keypoint
