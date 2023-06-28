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
#include "Camera.h"
namespace vslam
{
Vec2d Camera::project(const Vec3d & p) const
{
  if (p.z() <= 0) {
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  }

  return {(fx() * p(0) / p(2)) + cx(), (fy() * p(1) / p(2)) + cy()};
}

Vec3d Camera::reconstruct(const Vec2d & uv, double z) const
{
  return Vec3d((uv(0) - cx()) / fx() * z, (uv(1) - cy()) / fy() * z, z);
}

bool Camera::withinImage(const Vec2d & uv, double border) const
{
  const int bh = std::max<int>(0, (int)border * _h);
  const int bw = std::max<int>(0, (int)border * _w);

  return (bw < uv(0) && uv(0) < _w - bw && bh < uv(1) && uv(1) < _h - bh);
}

Camera::Camera(double fx, double fy, double cx, double cy, int width, int height)
: _w(width), _h(height)
{
  _K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  _Kinv << 1 / fx, 0, -cx / fx, 0, 1 / fy, -cy / fy, 0, 0, 1;
}

Camera::ShPtr Camera::resize(Camera::ConstShPtr cam, double s)
{
  return std::make_shared<Camera>(
    cam->fx() * s, cam->fy() * s, cam->cx() * s, cam->cy() * s, cam->width() * s,
    cam->height() * s);
}
std::string Camera::toString() const
{
  return format(
    "Focal Length: {},{} | Principal Point: {},{} | Resolution: {},{}", fx(), fy(), cx(), cy(), _w,
    _h);
}

}  // namespace vslam
