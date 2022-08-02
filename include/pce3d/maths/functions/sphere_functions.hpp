#ifndef sphere_functions_hpp
#define sphere_functions_hpp



#include <stdexcept>
#include <exception>
#include <cmath>
#include <algorithm>

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

namespace pce {
namespace maths {

glm::dvec3 calculateClosestPointVectorIntersectsSphere(
                            const glm::dvec3& v_origin,
                            const glm::dvec3& v_direction,
                            const glm::dvec3& sphere_center,
                            const double sphere_radius);
 



}
}



#endif /* sphere_functions_hpp */
