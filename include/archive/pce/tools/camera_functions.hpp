#ifndef camera_functions_hpp
#define camera_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist camera operation
-----------------------------------------------------------------*/

#include <glm/ext/quaternion_double.hpp>


namespace pce {
namespace camera {

static double HOP_ANGLE = 0.1;

/* update camera parameters */
void updateCameraRotationVersor(const double y_angle, const double xz_angle, 
                                const double rotation_direction
                                glm::dquat& output_rotation_versor);
  

/* functions below still need inputs */
void moveCameraDirectionLateral();
void moveCameraDirectionVertical();


}
}





#endif /* camera_functions_hpp */
