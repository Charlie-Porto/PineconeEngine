#ifndef rounding_functions_hpp
#define rounding_functions_hpp

#include <glm/vec3.hpp>
#include <algorithm>


namespace pce3d {
namespace round {

double roundDoubleToNearestInterval(const double interval, const double num);

glm::dvec3 roundVec3ComponentsToNearestInterval(const double interval, const glm::dvec3& vec);

} /* round */
} /* pce3d */




#endif /* rounding_functions_hpp */
