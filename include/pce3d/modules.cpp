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
#include "maths/functions/implementations/vector_functions.cpp"
#include "maths/functions/implementations/vertex_functions.cpp"
#include "maths/functions/implementations/pce_psuedo_randomness.cpp"
#include "maths/functions/implementations/rounding_functions.cpp"
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
#include "systems/functions/implementations/physics2Functions.cpp"
#include "systems/functions/implementations/massMapFunctions.cpp"
#include "systems/functions/implementations/collisionFunctions.cpp"
#include "systems/functions/implementations/rigidObjectFunctions.cpp"
#include "systems/objects/implementations/OrderRenderListNode.cpp"
#include "systems/objects/implementations/FaceOrderRenderNode.cpp"
#include "systems/objects/implementations/FaceOrderRenderHeadNode.cpp"

/* entity forging */
#include "entity_forging/implementations/sphere_forging.cpp"
#include "entity_forging/implementations/sheet_forging.cpp"
#include "entity_forging/implementations/rectangular_prism_forging.cpp"
#include "entity_forging/implementations/triangle_pyramid_forging.cpp"
#include "entity_forging/implementations/square_pyramid_forging.cpp"
#include "entity_forging/implementations/base_entity_forging.cpp"
#include "entity_forging/implementations/panel_forging.cpp"
#include "entity_forging/functions/implementations/forge_functions.cpp"
#include "entity_forging/factories/implementations/SphereFactory.cpp"
#include "entity_forging/special_entities/implementations/transparent_sheetbox_forging.cpp"




#endif /* modules_cpp */
