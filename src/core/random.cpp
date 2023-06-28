#include "macros.h"
#include "random.h"
namespace vslam::random
{
static std::default_random_engine eng(0);

template <typename T = double>
T U(T min, T max)
{
  std::uniform_real_distribution<double> distr(min, max);
  return static_cast<T>(distr(eng));
}
double U(double min, double max) { return U<double>(min, max); }
uint64_t U(uint64_t min, uint64_t max) { return U<uint64_t>(min, max); }

}  // namespace vslam::random
