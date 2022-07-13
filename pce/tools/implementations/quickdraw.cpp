#ifndef quickdraw_cpp
#define quickdraw_cpp

#include "../quickdraw.hpp"
#include "../SDL_cartesian_conversion.hpp"
#include "../render_functions.hpp"


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


void drawLine(const glm::dvec2& point_a, const glm::dvec2& point_b, const std::vector<int>& color) {
  const glm::dvec2 sdl_pixel_a = pce::convert::convertCartesianCoordinatesToSDL(point_a);
  const glm::dvec2 sdl_pixel_b = pce::convert::convertCartesianCoordinatesToSDL(point_b);
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  SDL_RenderDrawLine(
    Simulation::renderer,
    sdl_pixel_a.x, sdl_pixel_a.y,
    sdl_pixel_b.x, sdl_pixel_b.y
  );                                           
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}


void drawCircle(const glm::vec2& center_point, const std::vector<int>& color, double radius) {
  const glm::vec2 sdl_transform = pce::convert::convertCartesianCoordinatesToSDL(center_point);
  render::renderCircle(sdl_transform.x, sdl_transform.y, radius, color);
}



void drawSetOfEdges(const std::vector<std::pair<glm::dvec2, glm::dvec2>>& edges, 
                    const std::vector<int>& color) {
   
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);

  for (auto const& edge : edges) {
    const glm::dvec2 sdl_vertex_a = pce::convert::convertCartesianCoordinatesToSDL(edge.first);
    const glm::dvec2 sdl_vertex_b = pce::convert::convertCartesianCoordinatesToSDL(edge.second);

    SDL_RenderDrawLine(
      Simulation::renderer,
      sdl_vertex_a.x, sdl_vertex_a.y,
      sdl_vertex_b.x, sdl_vertex_b.y
    );                                           

  }

  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);

}

   
}
}




#endif /* quickdraw_cpp */
