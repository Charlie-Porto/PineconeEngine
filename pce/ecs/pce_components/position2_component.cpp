#ifndef position2_component_cpp
#define position2_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing an entity's position in 2space
-----------------------------------------------------------------*/

#include <glm/vec2.hpp>

namespace pce {

struct Position2 {
  glm::dvec2 actual;
  glm::dvec2 rotated_by_camera_angle_offset;
};

}

#endif /* position_component_cpp */
