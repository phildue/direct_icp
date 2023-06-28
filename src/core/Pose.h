// Copyright 2022 Philipp.Duernay
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef VSLAM_POSE_H__
#define VSLAM_POSE_H__
#include <memory>

#include "types.h"
namespace vslam
{
class Pose
{
public:
  typedef std::shared_ptr<Pose> ShPtr;
  typedef std::unique_ptr<Pose> UnPtr;
  typedef std::shared_ptr<const Pose> ConstShPtr;
  typedef std::unique_ptr<const Pose> ConstUnPtr;

  Pose(const SE3d & pose = SE3d(), const Mat6d & cov = Mat6d::Identity()) : _pose(pose), _cov(cov)
  {
  }

  const Vec3d & translation() const { return _pose.translation(); }
  Vec3d & translation() { return _pose.translation(); }
  double totalRotationDegrees() const
  {
    return _pose.log().block(3, 0, 3, 1).norm() * 180.0 / M_PI;
  }

  const SE3d & pose() const { return _pose; }
  SE3d & pose() { return _pose; }
  const SE3d & SE3() const { return _pose; }
  SE3d & SE3() { return _pose; }

  Vec6d log() const { return _pose.log(); }
  Matd<6, 6> & cov() { return _cov; }
  const Matd<6, 6> & cov() const { return _cov; }
  Pose inverse() const;

private:
  SE3d _pose;
  Matd<6, 6> _cov;
};
Pose operator*(const Pose & p1, const Pose & p0);

}  // namespace vslam
#endif
