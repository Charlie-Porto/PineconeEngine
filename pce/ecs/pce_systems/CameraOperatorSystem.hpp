#ifndef CameraOperatorSystem_hpp
#define CameraOperatorSystem_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class for operating the camera
-----------------------------------------------------------------*/
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

#include "../ecs_implementation/System.cpp"

#include "../../tools/virtual_keyboard.cpp"
#include "../../tools/camera.cpp"


namespace pce {
class CameraOperatorSystem : public ISystem {
public:
  void Init();

  const glm::dquat& ProvideCameraVersor() const;
  const glm::dvec3& ProvideCameraFocusPosition() const;
  const double ProvideCameraPositionScalar() const;

  void UpdateCamera();

private:
  Camera camera_;
  VirtualKeyboard keyboard_;

};
}
#endif /* CameraOperatorSystem_hpp */
