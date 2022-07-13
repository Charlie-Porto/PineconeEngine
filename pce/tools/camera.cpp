#ifndef camera_cpp
#define camera_cpp

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp> 

struct Camera {
  glm::dvec3 position;
  glm::dvec3 direction_of_view;
  glm::dvec3 point_of_focus; // in origin-focus mode, always the origin
  double position_scalar;
  double xz_angle;
  double xz_circle_radius;
  double y_angle;
  double focus_distance;
  bool in_free_roam_mode;
  bool in_point_focus_mode;
  glm::dquat rotation_versor;
};





#endif /* camera_cpp */
