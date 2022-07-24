#ifndef shadeFunctions_hpp
#define shadeFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the shade system
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>

namespace pce3d {
namespace shade {

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec);
/* returns a double between 0 and 2 */
/* 0 = black */
/* 1 = pure natural color */
/* 2 = white */



}
}




#endif /* shadeFunctions_hpp */
