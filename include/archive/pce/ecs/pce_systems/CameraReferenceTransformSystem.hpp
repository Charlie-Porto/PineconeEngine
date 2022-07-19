#ifndef CameraReferenceTransformSystem_hpp
#define CameraReferenceTransformSystem_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system that updates objects' camera-relative position, based on the 
position and rotation of the camera.
-----------------------------------------------------------------*/

#include "../ecs_implementation/System.cpp"

namespace pce {

class CameraReferenceTransformSystem : public ISystem {
public:
  CameraReferenceTransformSystem();
  void UpdateEntitiesCameraReferencePosition();

};


}





#endif /* CameraReferenceTransformSystem_hpp */
