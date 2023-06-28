
#include <experimental/filesystem>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/highgui.hpp>

#include "tum.h"
namespace fs = std::experimental::filesystem;

namespace vslam::evaluation::tum
{
vslam::Camera::ShPtr Camera()
{
  return std::make_shared<vslam::Camera>(525.0, 525.0, 319.5, 239.5, 640, 480);
}

cv::Mat convertDepthMat(const cv::Mat & depth_, float factor)
{
  cv::Mat depth(cv::Size(depth_.cols, depth_.rows), CV_32FC1);
  for (int u = 0; u < depth_.cols; u++) {
    for (int v = 0; v < depth_.rows; v++) {
      const ushort d = depth_.at<ushort>(v, u);
      depth.at<float>(v, u) =
        factor * static_cast<float>(d > 0 ? d : std::numeric_limits<ushort>::quiet_NaN());
    }
  }
  return depth;
}

cv::Mat loadDepth(const std::string & path)
{
  // tum depth format: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
  cv::Mat depth = cv::imread(path, cv::IMREAD_ANYDEPTH);
  if (depth.empty()) {
    throw std::runtime_error(format("Could not load depth from [{}]", path));
  }
  if (depth.type() != CV_16U) {
    throw std::runtime_error(format("Depth image loaded incorrectly from [{}].", path));
  }

  depth = convertDepthMat(depth, 0.0002);

  return depth;
}
cv::Mat loadIntensity(const std::string & path)
{
  cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);

  if (img.empty()) {
    throw std::runtime_error(format("Could not load image from [{}]", path));
  }
  return img;
}

DataLoader::DataLoader(const std::string & dataPath, const std::string & assocFile)
: _extractedDataPath(dataPath),
  _cam(tum::Camera())
{
  readAssocTextfile(format("{}/{}",dataPath,assocFile));
}


cv::Mat DataLoader::loadDepth(std::uint64_t fNo) const
{
  return tum::loadDepth(format("{}/{}", extracteDataPath(), pathsDepth()[fNo]));
}
cv::Mat DataLoader::loadIntensity(std::uint64_t fNo) const
{
  return tum::loadIntensity(format("{}/{}", extracteDataPath(), pathsImage()[fNo]));
}

void DataLoader::readAssocTextfile(std::string filename)
{
  if (!fs::exists(filename)) {
    throw std::runtime_error("Could not find file [" + filename + "]");
  }
  std::string line;
  std::ifstream in_stream(filename.c_str());
  if (!in_stream.is_open()) {
    std::runtime_error("Could not open file at: " + filename);
  }

  while (!in_stream.eof()) {
    std::getline(in_stream, line);
    std::stringstream ss(line);
    std::string buf;
    int c = 0;
    while (ss >> buf) {
      c++;
      if (c == 3) {
        buf.erase(std::remove(buf.begin(), buf.end(), '.'), buf.end());
        buf.erase(std::remove(buf.begin(), buf.end(), ' '), buf.end());
        const long td = std::stol(format("{}000", buf));
        _timestamps.push_back(td);
        _timestampsImage.push_back(td);
      } else if (c == 1) {
        buf.erase(std::remove(buf.begin(), buf.end(), '.'), buf.end());
        buf.erase(std::remove(buf.begin(), buf.end(), ' '), buf.end());
        const long td = std::stol(format("{}000", buf));
        _timestampsDepth.push_back(td);
      } else if (c == 2) {
        _depthFilenames.push_back(buf);
      } else if (c == 4) {
        _imgFilenames.push_back(buf);
      }
    }
  }
  in_stream.close();
}

double DataLoader::duration(int from, int to) const
{
  return static_cast<double>(
           timestamps()[std::min<int>(to, timestamps().size() - 1)] -
           timestamps()[std::max<int>(from, 0)]) /
         1e9;
}

}  // namespace vslam::evaluation::tum
