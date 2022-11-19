#ifndef objectRenderFunctions_hpp
#define objectRenderFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
high-level functions for rendering objects. Main purpose of this module is 
to help organize the Render System module.
-----------------------------------------------------------------*/

#include <iostream>
#include <pcecs/ecs/System.cpp>
#include "../../utilities/functions/quickdraw.hpp"
#include "../../utilities/functions/render_functions.hpp"
#include "../../maths/objects/Quadrilateral.hpp"
#include "../../maths/objects/Triangle.hpp"
#include "../functions/renderFunctions.hpp"

namespace pce3d {
namespace render {

void renderSphereObject(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
  , pce::Position position
);


void renderCylinder(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
);


void renderPolyhedron(
    pce::Surface surface
  , pce::FaceShade shade
  , pce::RigidObject rigid_object
);


}
}





#endif /* objectRenderFunctions_hpp */
