#ifndef VSLAM_EVALUATION_H__
#define VSLAM_EVALUATION_H__
#include <string>
namespace vslam::evaluation
{
void computeRuntimeKPIs(const std::string & file);
void computeKPIs(const std::string & sequenceId, const std::string & experimentId, bool upload);

}  // namespace vslam::evaluation
#endif  //VSLAM_EVALUATION_H__