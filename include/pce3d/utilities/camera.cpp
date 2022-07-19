#ifndef camera_cpp
#define camera_cpp

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

namespace pce3d {

struct Camera {
  glm::dvec3 position;
  glm::dvec3 view_direction;
  double focus_distance;
  double zoom_amount;
  double lense_curvature;
  double lense_width;
  glm::dquat rotation_versor;
};

}



#endif /* camera_cpp */
