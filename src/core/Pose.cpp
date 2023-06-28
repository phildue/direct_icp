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

#include "Pose.h"
namespace vslam
{
Pose Pose::inverse() const
{
  //See: Characterizing the Uncertainty of Jointly Distributed Poses in the Lie Algebra
  auto T_inv = _pose.inverse();
  auto sigma_inv = T_inv.Adj() * cov() * T_inv.Adj().transpose();
  return Pose(T_inv, sigma_inv);
}

Pose operator*(const Pose & p1, const Pose & p0)
{
  auto T_01 = p1.SE3() * p0.SE3();
  auto sigma_ij = p0.cov();
  auto adj_ij = p0.SE3().Adj();
  auto sigma_jk = p1.cov();
  /*
  Assuming the poses are independent
  See: Characterizing the Uncertainty of Jointly Distributed Poses in the Lie Algebra
  */
  auto sigma_01 = sigma_ij + adj_ij * sigma_jk * adj_ij;
  return Pose(T_01, sigma_01);
}
}  // namespace vslam
