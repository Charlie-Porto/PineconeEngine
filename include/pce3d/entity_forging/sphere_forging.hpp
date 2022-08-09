#ifndef sphere_forging_hpp
#define sphere_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for creating sphere entities
-----------------------------------------------------------------*/
#include <cmath>
#include <vector>
#include <utility>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "../components/rigid_object_component.cpp"
#include "triangle_pyramid_forging.hpp"
#include "base_entity_forging.hpp"

extern ControlPanel control;

namespace pce3d {
namespace forge {


Entity forgeSphereEntity(const double radius, const glm::dvec3 location, const std::vector<int> color,
                         const glm::dvec3& velocity, const double gravitational_force);
  
void registerSphereEntityWithOrderRenderSystem(double size);

}
}




#endif /* sphere_forging_hpp */
