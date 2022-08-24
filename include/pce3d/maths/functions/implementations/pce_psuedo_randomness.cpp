#ifndef pce_psuedo_randomness_cpp
#define pce_psuedo_randomness_cpp

#include "../pce_psuedo_randomness.hpp"

namespace pce3d {
namespace random {


void set_srand() {
  srand(time(NULL));
}

int getRandomIntBetweenZeroAndInt(const int x) {
   return int(rand() % x);
}

int getCoinFlipResult() {
  return int(rand() % 2);
}

std::vector<int> getRandomColor() {
  return {
    rand() % 255, 
    rand() % 255, 
    rand() % 255, 
    255
  };
}

double getRandomDoubleBetweenZeroAndDouble(const double x) {
  int big_x = int(x) * 1000;
  int random_int = getRandomIntBetweenZeroAndInt(big_x);
  return double(random_int) / 1000.0;
}

double getRandomDoubleBetweenDoubles(const double x, const double y) {
  return (getRandomDoubleBetweenZeroAndDouble(y-x)) + x;
}


double getRandomSignedDoubleBetweenDoubles(const double x, const double y) {
  const double random_double_unsigned = getRandomDoubleBetweenDoubles(x, y);
  if (getCoinFlipResult() == 1) {
    return -random_double_unsigned;
  }
  return random_double_unsigned;
}


double getRandomDoubleByTargetWithVariance(const double target, const double variance)
{
  const double range = abs(target - variance);
  const double range_number = getRandomDoubleBetweenZeroAndDouble(range);
  const double coin_flip = getRandomDoubleBetweenZeroAndDouble(2.0);
  const double multiplier = (coin_flip == 0.0) ? 1.0 : 0.0;
  return target + (range_number * multiplier);
  return target + (range_number * 1.0);
}




double GetRandomSignInFormOfDouble()
{
  const int rnum = rand() % 2;
  const double signed_rnum = double(rnum) - 1.0;
  return pce::math::sign(signed_rnum);
}




double getRandomDoubleBetweenOneAndZero()
{
  const int big_random = rand() % 1000000;
  return (double(big_random) / 1000000.0) * GetRandomSignInFormOfDouble();
}




glm::dvec3 getRandomUnitVector3()
{
  return glm::dvec3(
    getRandomDoubleBetweenOneAndZero(),
    getRandomDoubleBetweenOneAndZero(),
    getRandomDoubleBetweenOneAndZero()
  );
}


}
}



#endif /* pce_psuedo_randomness_cpp */
