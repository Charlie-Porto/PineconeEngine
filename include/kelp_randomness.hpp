#ifndef kelp_randomness_hpp
#define kelp_randomness_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
Purpose: this module contains various functions for (psuedo-) random number generation.

Prerequisites: set_srand() must be called before random numbers can be generated. 
-----------------------------------------------------------------*/

#include <vector>

namespace kelp {
namespace random {


void set_srand();
int getRandomIntBetweenZeroAndInt(const int x);
int getCoinFlipResult();
std::vector<int> getRandomColor();
double getRandomDoubleBetweenZeroAndDouble(const double x);
double getRandomDoubleBetweenDoubles(const double x, const double y);
double getRandomSignedDoubleBetweenDoubles(const double x, const double y);


}
}



#endif /* kelp_randomness_hpp */
