#ifndef raster_functions_hpp
#define raster_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to rasterize shapes of certain types
-----------------------------------------------------------------*/

#include <vector>
#include <unordered_map>
#include <glm/vec2.hpp>
#include <cmath>


namespace pce {
namespace raster {


/* for Circle Drawing */
std::vector<glm::dvec2> getCircleOctet(int xc, int yc, int x, int y);

std::unordered_map<glm::dvec2, glm::dvec2> getCircleOctetPairs(int xc, int yc, int x, int y);

std::vector<glm::dvec2> getCircleRasterizationPoints(int xc, int yc, int r);

std::unordered_map<glm::dvec2, glm::dvec2> getCircleOutlinePixelPairs(int xc, int yc, int r);

  
}
}




#endif /* raster_functions_hpp */
