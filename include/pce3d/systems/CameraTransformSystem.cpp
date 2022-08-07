#ifndef CameraTransformSystem_cpp
#define CameraTransformSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to adjust entities' camera-relative positions
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

#include "pcecs/ecs/System.cpp"

#include "../maths/functions/quaternion_functions.hpp"

extern ControlPanel control;

namespace pce3d {
class CameraTransformSystem : public ISystem {
public:

  void UpdateEntities(const glm::dvec3& transform_vector, const glm::dquat& versor) {
    for (auto const& entity : entities) {
      auto& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& position = control.GetComponent<pce::Position>(entity);

      const glm::dvec3 transformed_position = position.actual_center_of_mass - transform_vector;
      position.center_of_mass_relative_to_camera 
          = pce::rotateVector3byQuaternion(transformed_position, versor);

      for (auto const& [id, vertex] : rigid_object.vertices) {
        rigid_object.camera_transformed_vertices[id] = vertex - transform_vector;
        rigid_object.camera_transformed_vertices[id] 
            = pce::rotateVector3byQuaternion(rigid_object.camera_transformed_vertices.at(id), versor);
      }
      for (auto const& [id, corner] : rigid_object.face_corner_map) {
        rigid_object.camera_rotated_face_corner_map[id] = corner - transform_vector;
        rigid_object.camera_rotated_face_corner_map[id] 
            = pce::rotateVector3byQuaternion(rigid_object.camera_rotated_face_corner_map.at(id), versor);
      }
    }
  }

};
}
#endif /* CameraTransformSystem_cpp */
