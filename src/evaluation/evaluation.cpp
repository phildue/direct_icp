#include "core/types.h"
#include "evaluation.h"
#define SCRIPT_DIR "/home/ros/vslam_ros/src/vslam/script/"
namespace vslam::evaluation
{
void computeRuntimeKPIs(const std::string & file)
{
  const int ret = system(format(
                           "python3 " SCRIPT_DIR "vslampy/plot/parse_performance_log.py "
                           "--file {}",
                           file)
                           .c_str());
  if (ret != 0) {
    throw std::runtime_error("Running evaluation script failed!");
  }
}

void computeKPIs(const std::string & sequence_id, const std::string & experiment_id, bool upload)
{
  const int ret = system(format(
                           "python3 " SCRIPT_DIR "vslampy/evaluation/evaluation.py "
                           "--sequence_id {} --experiment_name {} {}",
                           sequence_id, experiment_id, upload ? "--upload" : "")
                           .c_str());
  if (ret != 0) {
    throw std::runtime_error("Running evaluation script failed!");
  }
}
}  // namespace vslam::evaluation