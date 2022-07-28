#ifndef CameraOperatorSystem_cpp
#define CameraOperatorSystem_cpp


#include <pcecs/ecs/System.cpp>
#include "functions/cameraOperatorFunctions.hpp"
#include "../utilities/objects/virtual_keyboard.hpp"
#include "../utilities/camera_trolley.hpp"

extern ControlPanel control;

namespace pce3d {
class CameraOperatorSystem : public ISystem {
public:
  CameraOperatorSystem() {
    keyboard_ = pce::VirtualKeyboard();
    trolley_ = pce3d::CameraTrolley{
      .movement_speed = 0.5,
      .swivel_speed = 3.0,
      .y_angle = 0.0,
      .xz_angle = 0.0
    };
  }

  void PrintCameraParameters(const Camera& camera) {
    ezp::print_item(camera.view_direction.x);
    ezp::print_item(camera.view_direction.y);
    ezp::print_item(camera.view_direction.z);
  }

  void UpdateCamera(Camera& camera) {
    pce3d::camera::pollVirtualKeyboard(keyboard_, camera, trolley_);
    pce3d::camera::calculateUpdatedCameraRotationVersor(trolley_.y_angle, trolley_.xz_angle, 
                                                        camera.rotation_versor);

    // PrintCameraParameters(camera);
  }

private:
  pce::VirtualKeyboard keyboard_;
  pce3d::CameraTrolley trolley_;


};
}
#endif /* CameraOperatorSystem_cpp */
