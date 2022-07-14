#ifndef position3_component_cpp
#define position3_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing an entity's position in 3space
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>

namespace pce {

struct Position3 {
  glm::dvec3 actual;
  glm::dvec3 camera_relative;
};

}

#endif /* position_component_cpp */
