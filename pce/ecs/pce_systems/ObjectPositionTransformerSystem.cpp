#ifndef ObjectPositionTransformerSystem_cpp
#define ObjectPositionTransformerSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system that rotates and transforms object positions in order for them
to be properly detected & rendered by the radar and rendering systems.
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include <ezprint.cpp>
#include <vezprint.cpp>

#include "../System.cpp"

extern ControlPanel control;

namespace pce{
class ObjectPositionTransformerSystem : public ISystem {
public:

void UpdateEntities(const glm::dvec3& transform_vector, const glm::dquat& versor,
                    const glm::dvec3& camera_position) {
  for (auto const entity : entities) {

    /* update rotated position */
    auto& position = control.GetComponent<pce::Position>(entity);
    const glm::dvec3 transformed_position = position.actual - transform_vector;
    position.rotated = qfunc::rotateVector3byQuaternion(transformed_position, versor);
  }
  
}

private:
};
}
#endif /* ObjectPositionTransformerSystem_cpp */
