#ifndef camera_functions_cpp
#define camera_functions_cpp


/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions that operate on the Camera struct
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>
#include "../../../../tools/quaternion_functions.hpp"

namespace pce {
namespace camera {

auto const y_axis_unit_vector3 = glm::dvec3(0.0, 1.0, 0.0);
auto const x_axis_unit_vector3 = glm::dvec3(1.0, 0.0, 0.0);

glm::dquat getCameraVerticalRotationVersor(const double& y_angle) {
  glm::dquat vertical_rotation_versor = pce::convertAngleAxisToQuaternion(y_angle,
                                                                 x_axis_unit_vector3);
  return vertical_rotation_versor;
}

glm::dquat getCameraHorizontalRotationVersor(const double& xz_angle) {
  glm::dquat horizontal_rotation_vector = pce::convertAngleAxisToQuaternion(
                                              xz_angle,
                                              y_axis_unit_vector3);
  return horizontal_rotation_vector;
}

glm::dvec3 rotateVec3byProductOfTwoVersors(const glm::dquat& versor_a,
                                           const glm::dquat& versor_b,
                                           const glm::dvec3& vect) {
  glm::dquat versor_product = normalize(versor_a * versor_b);
  glm::dvec3 rotated_vector = pce::rotateVector3byQuaternion(vect, versor_product);
  return rotated_vector;
}


}
}




#endif /* camera_functions_cpp */
