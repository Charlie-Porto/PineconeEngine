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
#include <ezprint.cpp>

extern ControlPanel control;

namespace pce3d {
class CameraTransformSystem : public ISystem {
public:

  void UpdateEntities(const glm::dvec3& transform_vector, const glm::dquat& versor) {
    for (auto const& entity : entities) {

      auto& position = control.GetComponent<pce::Position>(entity);
      const glm::dvec3 transformed_position = position.actual_center_of_mass - transform_vector;
      position.center_of_mass_relative_to_camera 
          = pce::rotateVector3byQuaternion(transformed_position, versor);
      
    }
  }

private:
};
}
#endif /* CameraTransformSystem_cpp */
