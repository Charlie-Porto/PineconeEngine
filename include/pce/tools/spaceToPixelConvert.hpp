#ifndef spacePixelConversionFunctions_hpp
#define spacePixelConversionFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for converting 2d pixels to 3space
-----------------------------------------------------------------*/

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace pce {
namespace pixel_map {

const double PI = 3.14159265;


glm::dvec2 fastconvertPointOnViewSphereToPixel(const glm::dvec3& point,
                                               const glm::dvec3& view_sphere_center);


}
}



#endif /* spacePixelConversionFunctions_cpp */
