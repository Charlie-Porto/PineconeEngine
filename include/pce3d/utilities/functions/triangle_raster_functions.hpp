#ifndef triangle_raster_functions_hpp
#define triangle_raster_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for rasterizing triangles 
-----------------------------------------------------------------*/

#include "../../maths/objects/Triangle.hpp"

/* include Simulation.hpp for access to the renderer */
#include "../../../pceSDL/core/Simulation.hpp"

namespace pce3d {
namespace raster {


void sortTriangleVertices(maths::Triangle& triangle);
void rasterizeAndRenderTriangle(maths::Triangle& triangle, const std::vector<int>& color);
void rasterizeAndRenderTriangleTopHalf(maths::Triangle& triangle, const std::vector<int>& color);
void rasterizeAndRenderTriangleLowerHalf(maths::Triangle& triangle, const std::vector<int>& color);

  
}
}



#endif /* triangle_raster_functions_hpp */
