#ifndef local_rotation_component_cpp
#define local_rotation_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing the state of an entity's local rotation in 3space
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

namespace pce {

struct LocalRotation {
  double angle_x_axis;
  double angle_y_axis;
  double angle_z_axis;
  double angle;
  glm::dvec3 axis; 
  glm::dvec3 origin_of_rotation; // will be == to center of mass for rigid objects
  glm::dquat versor;
};

}

#endif /* local_rotation_component_cpp */
