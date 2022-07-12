#ifndef quickdraw_cpp
#define quickdraw_cpp

#include "quickdraw.h"
#include "tool_deps/SDL_cartesian_conversion.cpp"
#include "tool_deps/render_functions.h"


namespace pce {
namespace quickdraw {


void drawListOfPixels(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color) {
  std::vector<glm::dvec2> sdl_pixels;
  for (auto const& pixel : pixels) {
    sdl_pixels.push_back(pce::convert::convertCartesianCoordinatesToSDL(pixel));
  }
  render::renderPixelList(sdl_pixels, color);
}

void drawSinglePixel(const glm::dvec2& pixel, const std::vector<int>& color) {
  const glm::dvec2 sdl_pixel = convert::convertCartesianCoordinatesToSDL(pixel);
  render::renderPixel(sdl_pixel, color);
}






   
}
}




#endif /* quickdraw_cpp */
