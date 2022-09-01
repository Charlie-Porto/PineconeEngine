#ifndef cameraOperatorFunctions_cpp
#define cameraOperatorFunctions_cpp

#include <cmath>
#include "../cameraOperatorFunctions.hpp"
#include "../../../maths/functions/quaternion_functions.hpp"
#include <ezprint.cpp>

#include "../../../../pceSDL/core/CoreManager.hpp"

namespace pce3d {
namespace camera {

constexpr glm::dvec3 y_axis_unit_vector = glm::dvec3(0.0, 1.0, 0.0);
constexpr glm::dvec3 x_axis_unit_vector = glm::dvec3(1.0, 0.0, 0.0);
constexpr double PI = 3.14159265;


void calculateUpdatedCameraRotationVersor(const double y_angle, const double xz_angle,
                                          glm::dquat& rotation_versor) {
  const glm::dquat vertical_rotation_versor = pce::convertAngleAxisToQuaternion(
      y_angle, x_axis_unit_vector);
  const glm::dquat horizontal_rotation_versor = pce::convertAngleAxisToQuaternion(
      xz_angle, y_axis_unit_vector);
  // rotation_versor = vertical_rotation_versor * horizontal_rotation_versor;
  rotation_versor = horizontal_rotation_versor * vertical_rotation_versor;
}



void rotateCameraViewDirectionLaterally(CameraTrolley& trolley, const double direction,
                                        glm::dvec3& view_direction, const double focus_distance,
                                        const glm::dvec3& camera_position) {
  trolley.xz_angle += trolley.swivel_speed * direction;  

  const double xz_circle_radius = focus_distance * cos(PI * trolley.y_angle / 180.0);

  const double new_focus_x = xz_circle_radius * sin(PI * trolley.xz_angle / 180.0) + camera_position.x;
  const double new_focus_z = xz_circle_radius * cos(PI * trolley.xz_angle / 180.0) + camera_position.z;
  view_direction = glm::dvec3(new_focus_x, (view_direction.y + camera_position.y), new_focus_z) - camera_position;
}



void rotateCameraViewDirectionVertically(CameraTrolley& trolley, const double direction,
                                         glm::dvec3& view_direction, const double focus_distance,
                                         const glm::dvec3& camera_position) {
  trolley.y_angle += trolley.swivel_speed * direction;

  double new_focus_y = focus_distance * sin(PI * trolley.y_angle / 180.0) + camera_position.y;

  view_direction.y = new_focus_y;
  rotateCameraViewDirectionLaterally(trolley, 0.0, 
                                     view_direction, focus_distance,
                                     camera_position);
}


void moveCameraPositionLaterally(glm::dvec3& position, const glm::dvec3& view_direction, 
                                 const glm::dvec3& direction, const double speed) {
  const glm::dvec3 orthogonal_view_direction = glm::normalize(glm::dvec3(view_direction.z,
                                                                         0.0,
                                                                         -view_direction.x));
  auto const movement_vector = glm::dvec3(
    (direction.x * -orthogonal_view_direction.x + direction.z * view_direction.x),
    0.0,
    (direction.z * view_direction.z + -direction.x * orthogonal_view_direction.z)
  ) * speed * 6.0;
  position += movement_vector;
}


void moveCameraPositionUpDown(glm::dvec3& position, const double direction, const double speed) {
  position.y += direction * speed * 10.0;
}


void pollVirtualKeyboard(pce::VirtualKeyboard& keyboard, pce3d::Camera& camera,
                         pce3d::CameraTrolley& trolley) {
  pce::JoystickReport report = keyboard.check_buttons();

  if (report.R_pressed) { 
    rotateCameraViewDirectionLaterally(trolley, 1.0, camera.view_direction, 
                                       camera.focus_distance, camera.position);
  }
  if (report.L_pressed) { 
    rotateCameraViewDirectionLaterally(trolley, -1.0, camera.view_direction, 
                                       camera.focus_distance, camera.position);
  }
  if (report.Up_pressed) { 
    rotateCameraViewDirectionVertically(trolley, -1.0, camera.view_direction, 
                                        camera.focus_distance, camera.position);
  }
  if (report.Down_pressed) { 
    rotateCameraViewDirectionVertically(trolley, 1.0, camera.view_direction, 
                                        camera.focus_distance, camera.position); } if (report.A_pressed) { 
    moveCameraPositionLaterally(camera.position, camera.view_direction, 
                                glm::dvec3(-1, 0, 0), trolley.movement_speed);
  }
  if (report.D_pressed) { 
    moveCameraPositionLaterally(camera.position, camera.view_direction, 
                                glm::dvec3(1, 0, 0), trolley.movement_speed);
  }
  if (report.W_pressed) { 
    moveCameraPositionLaterally(camera.position, camera.view_direction, 
                                glm::dvec3(0, 0, -1), trolley.movement_speed/10.0);
  }
  if (report.S_pressed) { 
    moveCameraPositionLaterally(camera.position, camera.view_direction, 
                                glm::dvec3(0, 0, 1), trolley.movement_speed/10.0);
  }
  if (report.r_pressed) { 
    moveCameraPositionUpDown(camera.position, -1.0, trolley.movement_speed);
  }
  if (report.F_pressed) { 
    moveCameraPositionUpDown(camera.position, 1.0, trolley.movement_speed);
  }



}


void pollMouseAndUpdateViewAngle(pce::mouse::VirtualMouse& mouse, pce3d::Camera& camera,
                                 pce3d::CameraTrolley& trolley) {
  pce::MouseReport mouse_report = mouse.PollMouse();
  constexpr double mouse_movement_smoother = 20.0;
  constexpr double mouse_sensitivity = 1.0;
  const double direction_x_shift = (mouse_report.x_position - mouse_report.prev_x_pos) / mouse_movement_smoother;
  const double direction_y_shift = -(mouse_report.y_position - mouse_report.prev_y_pos) / mouse_movement_smoother;


  if (!mouse.just_moved) {
    if (direction_x_shift != 0 || direction_y_shift != 0) {
      rotateCameraViewDirectionLaterally(trolley, direction_x_shift * mouse_sensitivity, 
                                        camera.view_direction, camera.focus_distance, camera.position);
      rotateCameraViewDirectionVertically(trolley, -direction_y_shift * mouse_sensitivity, 
                                        camera.view_direction, camera.focus_distance, camera.position);
      SDL_WarpMouseInWindow(NULL, int(pce::CoreManager::SCREEN_X/2), int(pce::CoreManager::SCREEN_Y/2));
      mouse.just_moved = true;
    } else {
      mouse.just_moved = false;
    }
  } else {
    mouse.just_moved = false;
  }
}




}}



#endif /* cameraOperatorFunctions_cpp */
