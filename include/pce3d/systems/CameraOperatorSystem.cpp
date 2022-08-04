#ifndef CameraOperatorSystem_cpp
#define CameraOperatorSystem_cpp


#include <pcecs/ecs/System.cpp>
#include "functions/cameraOperatorFunctions.hpp"
#include "../utilities/objects/virtual_keyboard.hpp"
#include "../utilities/objects/mouse_controller.hpp"
#include "../utilities/camera_trolley.hpp"

extern ControlPanel control;

namespace pce3d {
class CameraOperatorSystem : public ISystem {
public:

  CameraOperatorSystem() {
    keyboard_ = pce::VirtualKeyboard();
    trolley_ = pce3d::CameraTrolley{
      .movement_speed = 0.4,
      .swivel_speed = 2.0,
      .y_angle = 0.0,
      .xz_angle = 0.0
    };
  }


  void UpdateCamera(Camera& camera) {
    // pce3d::camera::pollMouseAndUpdateViewAngle(mouse_, camera, trolley_);
    pce3d::camera::pollVirtualKeyboard(keyboard_, camera, trolley_);
    pce3d::camera::calculateUpdatedCameraRotationVersor(trolley_.y_angle, trolley_.xz_angle, 
                                                        camera.rotation_versor);
    
    std::cout << "camera position: " << camera.position.x << ", "<< camera.position.y << ", "<< camera.position.z << '\n';
    std::cout << "camera direction: " << camera.view_direction.x << ", "<< camera.view_direction.y << ", "<< camera.view_direction.z << '\n';
  }

private:
  pce::VirtualKeyboard keyboard_;
  pce::mouse::VirtualMouse mouse_;
  pce3d::CameraTrolley trolley_;


};
}
#endif /* CameraOperatorSystem_cpp */
