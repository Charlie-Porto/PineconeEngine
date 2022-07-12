#ifndef local_rotation2_component_cpp
#define local_rotation2_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing the state of an entity's rotation in 2space
-----------------------------------------------------------------*/

#include <glm/vec2.hpp>

namespace pce {

struct LocalRotation2 {
  double angle;
  glm::vec2 origin_of_rotation;
};

}

#endif /* local_rotation2_component_cpp */
