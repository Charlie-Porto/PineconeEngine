#ifndef force_component_cpp
#define force_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing information about a force acting on a entity
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>

namespace pce {

struct Force3 {
  glm::dvec3 direction;
  double newtons;
  glm::dvec3 point_of_contact;
};

}

#endif /* force_component_cpp */
