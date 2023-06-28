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
// Created by phil on 07.08.21.
//

#ifndef VSLAM_TYPES_H
#define VSLAM_TYPES_H

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <Eigen/Dense>
#include <limits>
#include <sophus/se3.hpp>

#define INFd std::numeric_limits<double>::infinity()
#define INFf std::numeric_limits<float>::infinity()
#define INFi std::numeric_limits<int>::infinity()
#define NANd std::numeric_limits<double>::quiet_NaN()
#define NANf std::numeric_limits<float>::quiet_NaN()
#define NANi std::numeric_limits<int>::quiet_NaN()

using fmt::format;
using fmt::print;

namespace vslam
{
typedef std::uint8_t image_value_t;
typedef Eigen::Matrix<image_value_t, Eigen::Dynamic, Eigen::Dynamic> Image;
typedef std::vector<Image> ImageVec;

typedef double depth_value_t;
typedef Eigen::Matrix<depth_value_t, Eigen::Dynamic, Eigen::Dynamic> DepthMap;
typedef std::vector<DepthMap> DepthMapVec;

typedef std::uint64_t Timestamp;
typedef Sophus::SE3d SE3d;
typedef Sophus::SE3f SE3f;

typedef Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic> MatXui8;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatXi;
typedef std::vector<MatXi> MatXiVec;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXd;
typedef std::vector<MatXd> MatXdVec;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXf;

template <typename Derived, int nRows, int nCols>
using Mat = Eigen::Matrix<Derived, nRows, nCols>;

template <int nRows, int nCols>
using Matd = Eigen::Matrix<double, nRows, nCols>;

template <int nRows, int nCols>
using Matf = Eigen::Matrix<float, nRows, nCols>;

typedef Eigen::VectorXd VecXd;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Matrix<double, 2, 2> Mat2d;
typedef Eigen::Matrix<double, 3, 3> Mat3d;
typedef Eigen::Matrix<double, 3, 3> Mat4d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 6, 6> Mat6d;
typedef Eigen::Matrix<double, 12, 1> Vec12d;
typedef Eigen::Matrix<double, 12, 12> Mat12d;

typedef Eigen::VectorXf VecXf;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix<float, 2, 2> Mat2f;
typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 6, 6> Mat6f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 12, 12> Mat12f;

typedef double num_t;
typedef Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic> MatX;
typedef Mat<num_t, 3, 3> Mat3;
typedef Mat<num_t, 2, 2> Mat2;
typedef Mat<num_t, -1, 1> VecX;
typedef Mat<num_t, 2, 1> Vec2;
typedef Mat<num_t, 3, 1> Vec3;
typedef Mat<num_t, 4, 1> Vec4;
typedef Mat<num_t, 6, 1> Vec6;
typedef Mat<num_t, 6, 6> Mat6;
typedef Mat<num_t, 12, 1> Vec12;
typedef Mat<num_t, 12, 12> Mat12;
typedef std::vector<MatX> MatXVec;

typedef std::uint64_t Timestamp;

}  // namespace vslam

#endif  //VSLAM_TYPES_H
