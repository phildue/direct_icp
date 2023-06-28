



#include <thread>
#include <iostream>
#include "direct_icp/DirectIcp.h"
#include "evaluation/evaluation.h"
#include "evaluation/tum.h"
using namespace vslam;

int main(int argc, char ** argv)
{
  std::string sequenceId = "assoc.txt";
  std::string root = "/data/rgbd_dataset_freiburg2_desk";
  
  if(argc == 3){
  root = argv[1];
  sequenceId = argv[2];
  }else{
     std::cerr << "Usage: main_tum [data folder] [assoc.txt]" << std::endl;
     return -1;
  }
  auto dl = std::make_unique<evaluation::tum::DataLoader>(root, sequenceId);

  auto directIcp = std::make_shared<DirectIcp>(DirectIcp::defaultParameters());

  const size_t fEnd = dl->timestamps().size();
  Pose motion;
  Pose pose;
  cv::Mat img0 = dl->loadIntensity(0), depth0 = dl->loadDepth(0);
  for (size_t fId = 0; fId < fEnd; fId++) {
    try {
      
      const cv::Mat img = dl->loadIntensity(fId);
      const cv::Mat depth = dl->loadDepth(fId);
      
      motion = directIcp->computeEgomotion(dl->cam(), img0, depth0, img, depth, motion.SE3());
      
      pose = motion * pose;
      img0 = img;
      depth0 = depth;
      const SE3d se3 = pose.SE3().inverse();
      const Vec3d t = se3.translation();
      const Eigen::Quaterniond q = se3.unit_quaternion();

      print("{:.5f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n", dl->timestamps()[fId]/1e9, t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());

    } catch (const std::runtime_error & e) {
      std::cerr << e.what() << std::endl;
    }

  }
}
