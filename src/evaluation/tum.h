#ifndef VSLAM_TUM_H__
#define VSLAM_TUM_H__
#include <memory>
#include <string>
#include <vector>

#include "core/Camera.h"
#include "core/types.h"
namespace vslam::evaluation::tum
{

cv::Mat convertDepthMat(const cv::Mat & depth, float factor = 0.0002);
cv::Mat loadDepth(const std::string & path);
cv::Mat loadIntensity(const std::string & path);

vslam::Camera::ShPtr Camera();

class DataLoader
{
public:
  typedef std::shared_ptr<DataLoader> ShPtr;
  typedef std::unique_ptr<DataLoader> UnPtr;
  typedef std::shared_ptr<const DataLoader> ConstShPtr;
  typedef std::unique_ptr<const DataLoader> ConstUnPtr;

  DataLoader(const std::string & dataPath, const std::string & assocFile);

  //Frame::UnPtr loadFrame(std::uint64_t fNo) const;
  cv::Mat loadDepth(std::uint64_t fNo) const;
  cv::Mat loadIntensity(std::uint64_t fNo) const;

  size_t nFrames() const { return _timestamps.size(); }
  Camera::ConstShPtr cam() const { return _cam; }
  std::string extracteDataPath() const { return _extractedDataPath; }

  const std::vector<std::string> & pathsImage() const { return _imgFilenames; }
  const std::vector<std::string> & pathsDepth() const { return _depthFilenames; }
  const std::vector<Timestamp> & timestamps() const { return _timestamps; }
  const std::vector<Timestamp> & timestampsImage() const { return _timestampsImage; }
  const std::vector<Timestamp> & timestampsDepth() const { return _timestampsDepth; }

  double duration(int from = 0, int to = std::numeric_limits<int>::max()) const;

private:
  std::string _extractedDataPath;
  Camera::ShPtr _cam;
  std::vector<std::string> _imgFilenames, _depthFilenames;
  std::vector<Timestamp> _timestamps, _timestampsImage, _timestampsDepth;
  void readAssocTextfile(std::string filename);
};

}  // namespace vslam::evaluation::tum

#endif
