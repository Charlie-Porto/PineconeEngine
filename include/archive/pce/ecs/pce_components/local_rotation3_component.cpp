#ifndef local_rotation3_component_cpp
#define local_rotation3_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing the state of an entity's local rotation in 3space
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

namespace pce {

struct LocalRotation3 {
  double angle_x_axis;
  double angle_y_axis;
  double angle_z_axis;
  double angle;
  glm::dvec3 axis; 
  glm::dvec3 origin_of_rotation;
  glm::dquat versor;
};

}

#endif /* local_rotation3_component_cpp */
