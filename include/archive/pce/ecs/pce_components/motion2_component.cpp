#ifndef motion2_component_cpp
#define motion2_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component to store the state of an entity's motion
-----------------------------------------------------------------*/

#include <glm/vec2.hpp>

namespace pce {

struct Motion2 {
  double speed;
  glm::dvec2 travel_direction;

  /* members for airborne motion */
  bool if_airborne;
  double time_airborne;
  glm::dvec2 initial_velocity;
  glm::dvec2 previous_ground_position;
};

}

#endif /* motion2_component_cpp */
