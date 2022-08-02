#ifndef render_functions_hpp
#define render_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to render things
-----------------------------------------------------------------*/

#include <vector>
#include <glm/vec2.hpp>
#include "raster_functions.hpp"
// #include "SDL_cartesian_conversion.hpp"

/* include Simulation.h so that the renderer can be accessed */
#include "../../../pceSDL/core/Simulation.hpp"

extern glm::vec2 convertCartesianCoordinatesToSDL(glm::vec2 point);

namespace pce {
namespace render {

void renderPixelList(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color);

void renderPixel(const glm::dvec2& pixel, const std::vector<int>& color);

void renderCircle(int xc, int yc, int r, const std::vector<int>& color);

void renderFilledCircle(int xc, int yc, int r, const std::vector<int>& color);

void renderFilledCircleShaded(const PixelShadeMap& pixel_shade_map, const std::vector<int>& color);

void renderLine(const glm::dvec2& point_a, const glm::dvec2& point_b, const std::vector<int>& color);

void renderLineAsRendererIs(const glm::dvec2& point_a, const glm::dvec2& point_b);

void setRendererColor(std::vector<int> color);

void renderFilledTriangle();

}
}




#endif /* render_functions_hpp */
