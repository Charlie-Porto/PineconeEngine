#ifndef cameraOperatorFunctions_hpp
#define cameraOperatorFunctions_hpp

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "../../utilities/objects/virtual_keyboard.hpp"
#include "../../utilities/camera_trolley.hpp"


namespace pce3d {
namespace camera {


void calculateUpdatedCameraRotationVersor(const double y_angle, const double xz_angle,
                                          glm::dquat& rotation_versor);

void rotateCameraViewDirectionLaterally(CameraTrolley& trolley, const double direction, 
                                        glm::dvec3& view_direction, const double focus_distance, 
                                        const glm::dvec3& camera_position);


void rotateCameraViewDirectionVertically(CameraTrolley& trolley, const double direction,
                                         glm::dvec3& view_direction, const double focus_distance,
                                         const glm::dvec3& camera_position);

void moveCameraPositionLaterally(glm::dvec3& position, const glm::dvec3& view_direction,
                                 const glm::dvec3& direction, const double speed);
void moveCameraPositionUpDown(glm::dvec3& position, const double direction, const double speed);

void pollVirtualKeyboard(pce::VirtualKeyboard& keyboard, pce3d::Camera& camera,
                         pce3d::CameraTrolley& trolley);

}}




#endif /* cameraOperatorFunctions_hpp */
