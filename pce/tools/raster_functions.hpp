#ifndef raster_functions_hpp
#define raster_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to rasterize shapes of certain types
-----------------------------------------------------------------*/

#include <vector>
#include <glm/vec2.hpp>


namespace pce {
namespace raster {


/* for Circle Drawing */
std::vector<glm::dvec2> getCircleOctet(int xc, int yc, int x, int y);

std::vector<glm::dvec2> getCircleRasterizationPoints(int xc, int yc, int r);

  
}
}




#endif /* raster_functions_hpp */
