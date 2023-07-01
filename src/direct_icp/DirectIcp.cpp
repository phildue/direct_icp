

#include <execution>
#include <numeric>

#include "DirectIcp.h"

#include "keypoint_selection/keypoint_selection.h"


namespace vslam
{
std::map<std::string, double> DirectIcp::defaultParameters()
{
  return {
    {"nLevels", 4.0},
    {"minGradientIntensity", 5},
    {"minGradientDepth", 0.01},
    {"maxGradientDepth", 0.3},
    {"maxDepth", 5.0},
    {"maxIterations", 100},
    {"minParameterUpdate", 1e-4},
    {"maxErrorIncrease", 1.5},
    {"maxPoints", 640 * 480}};
}

DirectIcp::DirectIcp(const std::map<std::string, double> params)
: DirectIcp(
    params.at("nLevels"), params.at("minGradientIntensity"), params.at("minGradientDepth"),
    params.at("maxGradientDepth"), params.at("maxDepth"), params.at("maxIterations"),
    params.at("minParameterUpdate"), params.at("maxErrorIncrease"), params.at("maxPoints"))
{
}
DirectIcp::DirectIcp(
  int nLevels, double minGradientIntensity, double minGradientDepth, double maxGradientDepth,
  double maxZ, double maxIterations, double minParameterUpdate, double maxErrorIncrease,
  int maxPoints)
: _weightFunction(std::make_shared<TDistributionBivariate>(5.0, 1e-3, 10)),
  _nLevels(nLevels),
  _maxPoints(maxPoints),
  _minGradientIntensity(minGradientIntensity),
  _minGradientDepth(minGradientDepth),
  _maxGradientDepth(maxGradientDepth),
  _maxDepth(maxZ),
  _maxIterations(maxIterations),
  _minParameterUpdate(minParameterUpdate),
  _maxErrorIncrease(maxErrorIncrease)
{
}

Pose DirectIcp::computeEgomotion(
  Frame::ConstShPtr frame0, Frame::ConstShPtr frame1, const SE3d & guess)
{
  return computeEgomotion(frame0, frame1, Pose(guess, Mat6d::Identity() * INFd));
}

Pose DirectIcp::computeEgomotion(
  Camera::ConstShPtr cam, const cv::Mat & intensity0, const cv::Mat & depth0,
  const cv::Mat & intensity1, const cv::Mat & depth1, const SE3d & guess,
  const Mat6d & guessCovariance)
{
  auto f0 = std::make_shared<Frame>(intensity0, depth0, cam);
  f0->computePyramid(_nLevels);
  f0->computeDerivatives();
  f0->computePcl();
  auto f1 = std::make_shared<Frame>(intensity1, depth1, cam);
  f1->computePyramid(_nLevels);

  return computeEgomotion(f0, f1, Pose(guess, guessCovariance));
}

Pose DirectIcp::computeEgomotion(
  Frame::ConstShPtr frame0, Frame::ConstShPtr frame1, const Pose & prior)
{
  SE3f motion = prior.SE3().cast<float>();
  Mat6f covariance;
  for (_level = _nLevels - 1; _level >= 0; _level--) {
    const auto constraintsAll = selectConstraintsAndPrecompute(frame0, motion);

    double error = INFd;
    Vec6f dx = Vec6f::Zero();
    for (_iteration = 0; _iteration < _maxIterations; _iteration++) {

      auto constraintsValid = computeResidualsAndJacobian(constraintsAll, frame1, motion);

      if (constraintsValid.size() < 6) {
        motion = SE3f();
        break;
      }

      _weightFunction->computeWeights(constraintsValid);


      NormalEquations ne = computeNormalEquations(constraintsValid);

      if (prior.cov().allFinite()) {
        ne += computeNormalEquations(prior, motion);
      }

      if (ne.error / error > _maxErrorIncrease) {
        motion = SE3f::exp(dx) * motion;
        break;
      }
      error = ne.error;

      dx = ne.A.ldlt().solve(ne.b);

      motion = SE3f::exp(-dx) * motion;
      covariance = ne.A.inverse();

      if (dx.norm() < _minParameterUpdate) {
        break;
      }
    }
  }
  return Pose(motion.cast<double>(), covariance.cast<double>());
}

std::vector<DirectIcp::Constraint::ShPtr> DirectIcp::selectConstraintsAndPrecompute(
  Frame::ConstShPtr frame, const SE3f & motion) const
{
  const cv::Mat & intensity = frame->intensity(_level);
  const cv::Mat & depth = frame->depth(_level);
  const cv::Mat & dI = frame->dI(_level);
  const cv::Mat & dZ = frame->dZ(_level);

  std::vector<Vec2d> uv = keypoint::select(frame, _level, [&](const Vec2d & uv) {
    const float z = depth.at<float>(uv(1), uv(0));
    const cv::Vec2f dIuv = dI.at<cv::Vec2f>(uv(1), uv(0));
    const cv::Vec2f dZuv = dZ.at<cv::Vec2f>(uv(1), uv(0));
    return std::isfinite(z) && std::isfinite(dZuv[0]) && std::isfinite(dZuv[1]) && 0 < z &&
           z < _maxDepth && std::abs(dZuv[0]) < _maxGradientDepth &&
           std::abs(dZuv[1]) < _maxGradientDepth &&
           (std::abs(dIuv[0]) > _minGradientIntensity ||
            std::abs(dIuv[1]) > _minGradientIntensity || std::abs(dZuv[0]) > _minGradientDepth ||
            std::abs(dZuv[1]) > _minGradientDepth);
  });

  std::vector<Constraint::ShPtr> constraints(uv.size());
  std::transform(
    std::execution::par_unseq, uv.begin(), uv.end(), constraints.begin(),
    [&](const Vec2d & uv) -> Constraint::ShPtr {
      const float z = depth.at<float>(uv(1), uv(0));
      const float i = intensity.at<uint8_t>(uv(1), uv(0));

      const cv::Vec2f dIuv = dI.at<cv::Vec2f>(uv(1), uv(0));
      const cv::Vec2f dZuv = dZ.at<cv::Vec2f>(uv(1), uv(0));
      auto c = std::make_unique<Constraint>();
      c->uv0 = uv.cast<float>();
      c->iz0 = Vec2f(i, z);

      c->p0 = frame->p3d(uv(1), uv(0), _level).cast<float>();
      Mat<float, 2, 6> Jw = computeJacobianWarp(motion * c->p0, frame->camera(_level));
      c->J.row(0) = dIuv[0] * Jw.row(0) + dIuv[1] * Jw.row(1);
      c->JZJw = dZuv[0] * Jw.row(0) + dZuv[1] * Jw.row(1);
      return c;
    });

  constraints = keypoint::subsampling::uniform(
    constraints, frame->height(_level), frame->width(_level), _maxPoints,
    [](auto c) { return c->uv0; });

  return constraints;
}
Matf<2, 6> DirectIcp::computeJacobianWarp(const Vec3f & p, Camera::ConstShPtr cam) const
{
  const double & x = p.x();
  const double & y = p.y();
  const double z_inv = 1. / p.z();
  const double z_inv_2 = z_inv * z_inv;

  Matf<2, 6> J;
  J(0, 0) = z_inv;
  J(0, 1) = 0.0;
  J(0, 2) = -x * z_inv_2;
  J(0, 3) = y * J(0, 2);
  J(0, 4) = 1.0 - x * J(0, 2);
  J(0, 5) = -y * z_inv;
  J.row(0) *= cam->fx();
  J(1, 0) = 0.0;
  J(1, 1) = z_inv;
  J(1, 2) = -y * z_inv_2;
  J(1, 3) = -1.0 + y * J(1, 2);
  J(1, 4) = -J(1, 3);
  J(1, 5) = x * z_inv;
  J.row(1) *= cam->fy();

  return J;
}

std::vector<DirectIcp::Constraint::ShPtr> DirectIcp::computeResidualsAndJacobian(
  const std::vector<DirectIcp::Constraint::ShPtr> & constraints, Frame::ConstShPtr f1,
  const SE3f & motion) const
{
  /*Cache some constants for faster loop*/
  const SE3f motionInv = motion.inverse();
  const Camera::ConstShPtr cam = f1->camera(_level);
  const Mat3f K = cam->K().cast<float>();
  const Mat3f R = motion.rotationMatrix();
  const Vec3f t = motion.translation();
  const Mat3f Kinv = cam->Kinv().cast<float>();
  const Mat3f Rinv = motionInv.rotationMatrix();
  const Vec3f tinv = motionInv.translation();
  const cv::Mat & I1 = f1->I(_level);
  const cv::Mat & Z1 = f1->Z(_level);
  const float h = f1->height(_level);
  const float w = f1->width(_level);
  const int bh = std::max<int>(1, (int)(0.01f * h));
  const int bw = std::max<int>(1, (int)(0.01f * w));

  auto withinImage = [&](const Vec2f & uv) -> bool {
    return (bw < uv(0) && uv(0) < w - bw && bh < uv(1) && uv(1) < h - bh);
  };

  std::for_each(std::execution::par_unseq, constraints.begin(), constraints.end(), [&](auto c) {
    const Vec3f p0t = K * ((R * c->p0) + t);
    const Vec2f uv0t = Vec2f(p0t(0), p0t(1)) / p0t(2);

    const Vec2f iz1w = withinImage(uv0t) ? interpolate(I1, Z1, uv0t) : Vec2f::Constant(NANf);

    const Vec3f p1t = (Rinv * (iz1w(1) * (Kinv * Vec3f(uv0t(0), uv0t(1), 1.0)))) + tinv;

    c->residual = Vec2f(iz1w(0), p1t.z()) - c->iz0;

    c->J.row(1) = c->JZJw - computeJacobianSE3z(p1t);

    c->valid = p0t.z() > 0 && std::isfinite(iz1w.norm()) && std::isfinite(c->residual.norm()) &&
               std::isfinite(c->J.norm());
  });
  std::vector<Constraint::ShPtr> constraintsValid;
  std::copy_if(
    constraints.begin(), constraints.end(), std::back_inserter(constraintsValid),
    [](auto c) { return c->valid; });
  int idx = 0;
  std::for_each(
    constraintsValid.begin(), constraintsValid.end(), [&idx](auto c) { c->idx = idx++; });
  return constraintsValid;
}

DirectIcp::NormalEquations DirectIcp::computeNormalEquations(
  const std::vector<DirectIcp::Constraint::ShPtr> & constraints) const
{
  NormalEquations ne = std::transform_reduce(
    std::execution::par_unseq, constraints.begin(), constraints.end(),
    NormalEquations({Mat6f::Zero(), Vec6f::Zero(), 0.0, 0}), std::plus<NormalEquations>{},
    [](auto c) {
      return NormalEquations(
        {c->J.transpose() * c->weight * c->J, c->J.transpose() * c->weight * c->residual,
         c->residual.transpose() * c->weight * c->residual, 1});
    });

  return ne;
}

DirectIcp::NormalEquations DirectIcp::computeNormalEquations(
  const Pose & prior, const SE3f & motion)
{
  const Mat6f priorInformation = prior.cov().inverse().cast<float>();
  const Vec6f priorError =
    priorInformation * ((motion * prior.SE3().inverse().cast<float>()).log());
  return {priorInformation, priorError, priorError.norm(), 1};
}

Vec6f DirectIcp::computeJacobianSE3z(const Vec3f & p) const
{
  Vec6f J;
  J(0) = 0.0;
  J(1) = 0.0;
  J(2) = 1.0;
  J(3) = p(1);
  J(4) = -p(0);
  J(5) = 0.0;

  return J;
}

Vec2f DirectIcp::interpolate(
  const cv::Mat & intensity, const cv::Mat & depth, const Vec2f & uv) const
{
  auto sample = [&](int v, int u) -> Vec2f {
    const double z = depth.at<float>(v, u);
    return Vec2f(intensity.at<uint8_t>(v, u), std::isfinite(z) ? z : 0);
  };
  const double u = uv(0);
  const double v = uv(1);
  const double u0 = std::floor(u);
  const double u1 = std::ceil(u);
  const double v0 = std::floor(v);
  const double v1 = std::ceil(v);
  const double w_u1 = u - u0;
  const double w_u0 = 1.0 - w_u1;
  const double w_v1 = v - v0;
  const double w_v0 = 1.0 - w_v1;
  const Vec2f iz00 = sample(v0, u0);
  const Vec2f iz01 = sample(v0, u1);
  const Vec2f iz10 = sample(v1, u0);
  const Vec2f iz11 = sample(v1, u1);

  const double w00 = iz00(1) > 0 ? w_v0 * w_u0 : 0;
  const double w01 = iz01(1) > 0 ? w_v0 * w_u1 : 0;
  const double w10 = iz10(1) > 0 ? w_v1 * w_u0 : 0;
  const double w11 = iz11(1) > 0 ? w_v1 * w_u1 : 0;

  Vec2f izw = w00 * iz00 + w01 * iz01 + w10 * iz10 + w11 * iz11;
  izw /= w00 + w01 + w10 + w11;
  return izw;
}

DirectIcp::TDistributionBivariate::TDistributionBivariate(
  double dof, double precision, int maxIterations)
: _dof(dof), _precision(precision), _maxIterations(maxIterations)
{
}

void DirectIcp::TDistributionBivariate::computeWeights(
  const std::vector<Constraint::ShPtr> & features)
{
  VecXf weights = VecXf::Ones(features.size());
  std::vector<Mat2f> rrT(features.size());
  for (size_t n = 0; n < features.size(); n++) {
    rrT[n] = features[n]->residual * features[n]->residual.transpose();
  }

  for (int i = 0; i < _maxIterations; i++) {

    std::vector<Mat2f> wrrT(features.size());
    for (size_t n = 0; n < features.size(); n++) {
      wrrT[n] = weights(n) * rrT[n];
    }
    Mat2f sum = std::accumulate(rrT.begin(), rrT.end(), Mat2f::Zero().eval());

    const Mat2f scale_i = (sum / features.size()).inverse();

    const double diff = (_scale - scale_i).norm();
    _scale = scale_i;
    const double norm = _scale.norm();
    for (size_t n = 0; n < features.size(); n++) {
      weights(n) = computeWeight(features[n]->residual);
      features[n]->weight = weights(n) * _scale / norm;
    }

    if (diff < _precision) {
      break;
    }
  }
}
double DirectIcp::TDistributionBivariate::computeWeight(const Vec2f & r) const
{
  return (_dof + 2.0) / (_dof + r.transpose() * _scale * r);
}

}  // namespace vslam
