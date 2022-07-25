#ifndef radarFunctions_hpp
#define radarFunctions_hpp

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

namespace pce3d {
namespace radar {


glm::dvec2 convertPointOnViewSphereToPixel(glm::dvec3 point, bool is_center_of_gravity);

}
}



#endif /* radarFunctions_hpp */
