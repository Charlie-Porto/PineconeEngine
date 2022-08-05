#ifndef modules_cpp
#define modules_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
including not-yet-included implementation files
-----------------------------------------------------------------*/

/* maths */
#include "maths/functions/implementations/quaternion_functions.cpp"
#include "maths/functions/implementations/plane_functions.cpp"
#include "maths/functions/implementations/sphere_functions.cpp"
#include "maths/functions/implementations/triangle_functions.cpp"
#include "maths/functions/sign.hpp"

/* utilities */
#include "utilities/functions/implementations/quickdraw.cpp"
#include "utilities/functions/implementations/raster_functions.cpp"
#include "utilities/functions/implementations/render_functions.cpp"
#include "utilities/functions/implementations/triangle_raster_functions.cpp"
#include "utilities/functions/implementations/SDL_cartesian_conversion.cpp"
#include "utilities/objects/implementations/virtual_keyboard.cpp"

/* system functions */
#include "systems/functions/implementations/radarFunctions.cpp"
#include "systems/functions/implementations/renderFunctions.cpp"
#include "systems/functions/implementations/orderForRenderFunctions.cpp"
#include "systems/functions/implementations/cameraOperatorFunctions.cpp"
#include "systems/functions/implementations/shadeFunctions.cpp"
#include "systems/functions/implementations/spaceMapFunctions.cpp"
#include "systems/functions/implementations/physicsFunctions.cpp"

/* entity forging */
#include "entity_forging/implementations/sphere_forging.cpp"
#include "entity_forging/implementations/sheet_forging.cpp"
#include "entity_forging/implementations/rectangular_prism_forging.cpp"
#include "entity_forging/implementations/triangle_pyramid_forging.cpp"
#include "entity_forging/implementations/square_pyramid_forging.cpp"
#include "entity_forging/functions/implementations/forge_functions.cpp"



#endif /* modules_cpp */
