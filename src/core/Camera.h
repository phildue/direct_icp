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

//
// Created by phil on 30.06.21.
//

#ifndef VSLAM_CAMERA_H
#define VSLAM_CAMERA_H

#include <Eigen/Dense>
#include <memory>

#include "types.h"
namespace vslam
{
class Camera
{
public:
  using ConstShPtr = std::shared_ptr<const Camera>;
  using ShPtr = std::shared_ptr<Camera>;
  using Ptr = std::unique_ptr<Camera>;
  typedef std::vector<ConstShPtr> ConstShPtrVec;

  Camera(double fx, double fy, double cx, double cy, int width, int height);

  Vec2d project(const Vec3d & uv) const;
  Vec3d reconstruct(const Vec2d & uv, double z = 1.0) const;
  bool withinImage(const Vec2d & uv, double border = 0.) const;

  const double & fx() const { return _K(0, 0); }
  const double & fy() const { return _K(1, 1); }
  const int & width() const { return _w; }
  const int & height() const { return _h; }

  const double & cx() const { return _K(0, 2); }
  const double & cy() const { return _K(1, 2); }

  const Mat3d & K() const { return _K; }
  const Mat3d & Kinv() const { return _Kinv; }
  ShPtr static resize(ConstShPtr cam, double s);

  std::string toString() const;

private:
  Mat3d _K;     //< Intrinsic camera matrix
  Mat3d _Kinv;  //< Intrinsic camera matrix inverted
  int _w, _h;
};
}  // namespace vslam

#endif  //VSLAM_CAMERA_H
