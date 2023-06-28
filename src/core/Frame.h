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

#ifndef VSLAM_FRAME_H__
#define VSLAM_FRAME_H__
#include <memory>
#include <opencv2/core.hpp>

#include "Camera.h"
#include "Pose.h"
#include "types.h"
namespace vslam
{
class Frame
{
public:
  typedef std::shared_ptr<Frame> ShPtr;
  typedef std::shared_ptr<const Frame> ConstShPtr;
  typedef std::unique_ptr<Frame> UnPtr;
  typedef std::unique_ptr<const Frame> ConstUnPtr;
  typedef std::vector<ShPtr> VecShPtr;
  typedef std::vector<ConstShPtr> VecConstShPtr;

  Frame(
    const cv::Mat & gray, const cv::Mat & depth, Camera::ConstShPtr cam, const Timestamp & t = 0U,
    const Pose & pose = Pose());

  Frame(
    const cv::Mat & gray, Camera::ConstShPtr cam, const Timestamp & t = 0U,
    const Pose & pose = Pose());

  virtual ~Frame() = default;
  std::uint64_t id() const { return _id; }

  const cv::Mat & intensity(size_t level = 0) const { return _intensity.at(level); }
  const cv::Mat & I(size_t level = 0) const { return intensity(level); }
  const cv::Mat & dI(size_t level = 0) const;
  const cv::Mat & depth(size_t level = 0) const { return _depth.at(level); }
  const cv::Mat & Z(size_t level = 0) const { return depth(level); }
  const cv::Mat & dZ(size_t level = 0) const;
  Frame level(size_t level) const;

  const Pose & pose() const { return _pose; }
  Pose & pose() { return _pose; }

  const Timestamp & t() const { return _t; }
  Camera::ConstShPtr camera(size_t level = 0) const { return _cam.at(level); }
  size_t width(size_t level = 0) const { return _intensity.at(level).cols; }
  size_t height(size_t level = 0) const { return _intensity.at(level).rows; }
  size_t size(size_t level = 0) const { return width(level) * height(level); }

  size_t nLevels() const { return _intensity.size(); }
  bool withinImage(const Vec2d & pImage, double border = 7, size_t level = 0) const;

  Vec2d project(const Vec3d & pCamera, size_t level = 0) const;
  Vec3d reconstruct(const Vec2d & pImage, double depth = 1.0, size_t level = 0) const;
  Vec2d world2image(const Vec3d & pWorld, size_t level = 0) const;
  Vec3d image2world(const Vec2d & pImage, double depth = 1.0, size_t level = 0) const;

  void computeDerivatives();
  void computeIntensityDerivatives();
  void computeDepthDerivatives();
  void computePcl();
  void computePyramid(size_t nLevels);
  
  const Vec3d & p3d(int v, int u, size_t level = 0) const;
  Vec3d p3dWorld(int v, int u, size_t level = 0) const;
  std::vector<Vec3d> pcl(size_t level = 0, bool removeInvalid = false) const;
  std::vector<Vec3d> pclWorld(size_t level = 0, bool removeInvalid = false) const;

private:
  std::uint64_t _id;
  std::vector<cv::Mat> _intensity;
  std::vector<cv::Mat> _dI, _dZ;
  Camera::ConstShPtrVec _cam;
  Timestamp _t;
  Pose _pose;  //<< Pf = pose * Pw
  std::vector<cv::Mat> _depth;
  std::vector<std::vector<Vec3d>> _pcl;
    static std::uint64_t _idCtr;
};

}  // namespace vslam

#endif
