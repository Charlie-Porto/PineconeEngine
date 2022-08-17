#ifndef pce_psuedo_randomness_hpp
#define pce_psuedo_randomness_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
Purpose: this module contains various functions for (psuedo-) random number generation.

Prerequisites: set_srand() must be called before random numbers can be generated. 
-----------------------------------------------------------------*/

#include <cmath>
#include <vector>

namespace pce3d {
namespace random {


void set_srand();
int getRandomIntBetweenZeroAndInt(const int x);
int getCoinFlipResult();
std::vector<int> getRandomColor();
double getRandomDoubleBetweenZeroAndDouble(const double x);
double getRandomDoubleBetweenDoubles(const double x, const double y);
double getRandomSignedDoubleBetweenDoubles(const double x, const double y);
double getRandomDoubleByTargetWithVariance(const double target, const double variance);


}
}




#endif /* pce_psuedo_randomness_hpp */
