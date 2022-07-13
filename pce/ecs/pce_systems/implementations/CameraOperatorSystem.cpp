#ifndef CameraOperatorSystem_cpp
#define CameraOperatorSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class for operating the camera
-----------------------------------------------------------------*/

#include "../CameraOperatorSystem.hpp"
#include "functions/camera_functions.cpp"

namespace pce {
  

void CameraOperatorSystem::Init() {
  camera_.position = glm::dvec3(0.0, 0.0, 2000.0);
  camera_.point_of_focus = glm::dvec3(0.0, 0.0, 0.0);
  camera_.xz_angle = 0.0;
  camera_.y_angle = 0.0;
  camera_.position_scalar = camera_.position.z;
  camera_.xz_circle_radius = camera_.position_scalar;
  camera_.rotation_versor = glm::dquat(1.0, 0.0, 0.0, 0.0);
  camera_.in_point_focus_mode = true;
  camera_.in_free_roam_mode = false;
}


const glm::dquat& CameraOperatorSystem::ProvideCameraVersor() const {
  return camera_.rotation_versor;
}


const glm::dvec3& CameraOperatorSystem::ProvideCameraFocusPosition() const {
  return camera_.point_of_focus;
}


const double CameraOperatorSystem::ProvideCameraPositionScalar() const {
  return camera_.position_scalar;
}


void CameraOperatorSystem::UpdateCamera() {
  if (camera_.in_point_focus_mode) {
    // pce::camera::updateCameraPositionOriginFocus(camera_, keyboard_);
  } else if (camera_.in_free_roam_mode) {
    // pce::camera::updateCameraPositionFreeRoam(camera_, keyboard_);
  }
  ezp::print_labeled_item("camera position scalar: ", camera_.position_scalar);
}


}
#endif /* CameraOperatorSystem_cpp */
