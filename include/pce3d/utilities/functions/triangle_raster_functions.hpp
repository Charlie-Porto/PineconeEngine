#ifndef triangle_raster_functions_hpp
#define triangle_raster_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for rasterizing triangles 
-----------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include <glm/geometric.hpp>
#include "../../maths/functions/sign.hpp"
#include "quickdraw.hpp"
#include "../../maths/objects/Triangle.hpp"

/* include Simulation.hpp for access to the renderer */
#include "../../../pceSDL/core/Simulation.hpp"



namespace pce3d {
namespace raster {


void sortTriangleVertices(maths::Triangle& triangle);
void rasterizeAndRenderTriangle(const maths::Triangle& tri, const std::vector<int>& color);
void rasterizeAndRenderTriangleTopHalf(const maths::Triangle& triangle, const std::vector<int>& color);
void rasterizeAndRenderTriangleLowerHalf(const maths::Triangle& triangle, const std::vector<int>& color);

  
}
}



#endif /* triangle_raster_functions_hpp */
