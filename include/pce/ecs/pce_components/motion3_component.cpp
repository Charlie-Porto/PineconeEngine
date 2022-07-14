#ifndef motion3_component_cpp
#define motion3_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component to store the state of an entity's motion
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>

namespace pce {

struct Motion3 {
  double speed;
  glm::dvec3 travel_direction;

  /* members for airborne motion */
  bool if_airborne;
  double time_airborne;
  glm::dvec3 initial_velocity;
  glm::dvec3 previous_ground_position;
};

}

#endif /* motion3_component_cpp */
