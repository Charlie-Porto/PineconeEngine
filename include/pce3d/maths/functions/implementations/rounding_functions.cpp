#ifndef rounding_functions_cpp
#define rounding_functions_cpp

#include "../rounding_functions.hpp"


namespace pce3d {
namespace round {

double roundDoubleToNearestInterval(const double interval, const double num)
{
  // std::cout << "------" << '\n';
  double mutable_interval = std::max(interval, 0.1) * 10.0;
  double mutable_num = num * 10.0;
  // std::cout << "mutable_interval: " << mutable_interval << '\n';
  // std::cout << "mutable_num: " << mutable_num << '\n';
  int pow_ten = 1;
  while (mutable_interval != int(mutable_interval) && (mutable_num != int(mutable_num)))
  {
    /* this break is a safegaurd against errors & shenanigans */
    if (pow_ten > 6) { break; }

    mutable_interval *= 10.0;
    mutable_num *= 10.0;

    ++pow_ten;
  }


  // std::cout << "powten: " << pow_ten << '\n';
  mutable_interval = int(mutable_interval);
  mutable_num = int(mutable_num);
  // std::cout << "mutable_interval: " << mutable_interval << '\n';
  // std::cout << "mutable_num: " << mutable_num << '\n';

  const int remainder = int(mutable_num) % int(mutable_interval);
  const int low_bound = mutable_num - remainder;
  const int high_bound = low_bound + mutable_interval;
  // std::cout << "remainder: " << remainder << '\n';
  // std::cout << "high bound: " << high_bound << '\n';
  // std::cout << "low bound: " << low_bound << '\n';

  return ( double( (high_bound - mutable_num) <= (mutable_num - low_bound)
    ? high_bound : low_bound) / ( 10.0 * double(pow_ten)) );
}


glm::dvec3 roundVec3ComponentsToNearestInterval(const double interval, const glm::dvec3& vec)
{
  return glm::dvec3(
    roundDoubleToNearestInterval(interval, vec.x),
    roundDoubleToNearestInterval(interval, vec.y),
    roundDoubleToNearestInterval(interval, vec.z)
  );
}




} /* round */
} /* pce3d */


#endif /* rounding_functions_cpp */
