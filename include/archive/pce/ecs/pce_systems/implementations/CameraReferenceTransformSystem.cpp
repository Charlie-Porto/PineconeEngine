#ifndef CameraReferenceTransformSystem_cpp
#define CameraReferenceTransformSystem_cpp

#include "../CameraReferenceTransformSystem.hpp"
#include "../CameraOperatorSystem.hpp" // included for the camera versor
#include <ezprint.cpp>


namespace pce {

CameraReferenceTransformSystem::CameraReferenceTransformSystem() {
  ezp::print_item("creating CameraReferenceTransformSystem");
}


void CameraReferenceTransformSystem::UpdateEntitiesCameraReferencePosition() {
  for (auto const& entity : entities) {
    auto& position = control.GetComponent<pce::Position>(entity);
    const glm::dvec3 transformed_position = position.actual - pce::CameraOperatorSystem::camera_transformation;
    position.rotated = qfunc::rotateVector3byQuaternion(transformed_position, pce::CameraOperatorSystem::camera_versor);
  }
}


}



#endif /* CameraReferenceTransformSystem_cpp */
