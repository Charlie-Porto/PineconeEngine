#ifndef render_functions_cpp
#define render_functions_cpp

#include "../render_functions.hpp"
#include "../raster_functions.hpp"

namespace pce {
namespace render {


void renderPixel(const glm::dvec2& pixel, const std::vector<int>& color) {
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  SDL_RenderDrawPoint(Simulation::renderer, pixel.x, pixel.y);
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}


void renderPixelList(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color) {
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  for (int i = 0; i < pixels.size(); i++) {
    SDL_RenderDrawPoint(Simulation::renderer, pixels[i].x, pixels[i].y);
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}


void renderCircle(int xc, int yc, int r, const std::vector<int>& color) {
  const std::vector<glm::dvec2> points = raster::getCircleRasterizationPoints(xc, yc, r);
  pce::render::renderPixelList(points, color);
}

}
}



#endif /* render_functions_cpp */
