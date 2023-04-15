#ifndef render_functions_cpp
#define render_functions_cpp

#include "../render_functions.hpp"

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


void renderPixelList(const std::vector<glm::ivec2>& pixels, const std::vector<int>& color) {
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


void renderFilledCircle(int xc, int yc, int r, const std::vector<int>& color) {
  const std::vector<glm::dvec2> points = raster::getCircleRasterizationPoints(xc, yc, int(r)); 
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  for (auto const& point : points) {
    SDL_RenderDrawLine(Simulation::renderer, int(point.x), int(point.y), xc, yc);
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255); 
}

void renderFilledCircleShaded(const PixelShadeMap& pixel_shade_map, const std::vector<int>& color) {
  for (auto const& [pixel, shade_amount] : pixel_shade_map) {
    // the casting is necessary here
    SDL_SetRenderDrawColor(Simulation::renderer, int(double(color[0]) * pixel_shade_map.at(pixel)),
                                                 int(double(color[1]) * pixel_shade_map.at(pixel)),
                                                 int(double(color[2]) * pixel_shade_map.at(pixel)),
                                                 255);
    const glm::dvec2 p = convert::convertCartesianCoordinatesToSDL(pixel);
    SDL_RenderDrawPoint(Simulation::renderer, int(p.x), int(p.y));
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}

void renderFilledCircleShadedWithVirtualPixels(const PixelShadeMap& pixel_shade_map, const std::vector<int>& color,
                                               const int virtual_pixel_side_length, const PixelMap& outlinePixels)
{
  // std::cout << "rendering circle with virtual pixels: " << virtual_pixel_side_length <<'\n';
  for (auto const& [pixel, shade_amount] : pixel_shade_map)
  {
    SDL_SetRenderDrawColor(Simulation::renderer, int(double(color[0]) * shade_amount),
                                                 int(double(color[1]) * shade_amount),
                                                 int(double(color[2]) * shade_amount),
                                                 255);

    int ticks = 0;
    for (double i = pixel.y; ticks != (virtual_pixel_side_length + int(virtual_pixel_side_length * .66)); --i)
    {
      ++ticks;
      renderLineAsRendererIs(glm::dvec2(pixel.x, i), glm::dvec2(pixel.x + virtual_pixel_side_length, i));
    }
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}

void renderLine(const glm::dvec2& point_a, const glm::dvec2& point_b, const std::vector<int>& color) {
  const glm::dvec2 sdl_pixel_a = pce::convert::convertCartesianCoordinatesToSDL(point_a);
  const glm::dvec2 sdl_pixel_b = pce::convert::convertCartesianCoordinatesToSDL(point_b);
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  SDL_RenderDrawLine(Simulation::renderer, sdl_pixel_a.x, sdl_pixel_a.y,
                               sdl_pixel_b.x, sdl_pixel_b.y);
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255); 
}

void renderLineAsRendererIs(const glm::dvec2& point_a, const glm::dvec2& point_b) {
  const glm::dvec2 sdl_pixel_a = pce::convert::convertCartesianCoordinatesToSDL(point_a);
  const glm::dvec2 sdl_pixel_b = pce::convert::convertCartesianCoordinatesToSDL(point_b);
  SDL_RenderDrawLine(Simulation::renderer, sdl_pixel_a.x, sdl_pixel_a.y,
                                           sdl_pixel_b.x, sdl_pixel_b.y);
}

inline void renderPreSdlConvertedLineAsRendererIs(const glm::dvec2& point_a, const glm::dvec2& point_b)
{
  if (point_a.x != point_b.x || point_a.y != point_b.y)
  {
    SDL_RenderDrawLine(Simulation::renderer, int(point_a.x), int(point_a.y),
                                            int(point_b.x), int(point_b.y));
  }
}

inline void setRendererColor(std::vector<int> color) {
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
}


void renderQuadrilateralDiagonals(std::vector<glm::dvec2> points, std::vector<int> color)
{
  assert(points.size() == 4);
  std::vector<std::pair<glm::dvec2, glm::dvec2>> lines{};
  
  glm::dvec2 line_a_start_top_direction = points[3] - points[0];
  glm::dvec2 line_a_end_top_direction = points[3] - points[2];
  glm::dvec2 line_a_start_bottom_direction = points[3] - points[0];
  glm::dvec2 line_a_end_bottom_direction = points[3] - points[2];

  std::vector<glm::dvec2> directions = {
    glm::normalize(line_a_start_top_direction),
    glm::normalize(line_a_end_top_direction),
    glm::normalize(line_a_start_bottom_direction),
    glm::normalize(line_a_end_bottom_direction),
  };
  
   const double inc = 1.0;

  lines = {
    // std::make_pair(points[0] + inc * directions[0], points[2] + inc * directions[1]),
    // std::make_pair(points[0] + inc * directions[2], points[2] + inc * directions[3]),
    // std::make_pair(points[0] + inc * 2.0 * directions[0], points[2] + inc * 2.0 * directions[1]),
    // std::make_pair(points[0] + inc * 2.0 * directions[2], points[2] + inc * 2.0 * directions[3]),
    // std::make_pair(points[0] + inc * 3.0 * directions[0], points[2] + inc * 3.0 * directions[1]),
    // std::make_pair(points[0] + inc * 3.0 * directions[2], points[2] + inc * 3.0 * directions[3]),
    std::make_pair(points[0] + inc * directions[0], points[2] + inc * directions[1]),
    std::make_pair(points[0] + inc * directions[2], points[2] + inc * directions[3]),
    std::make_pair(points[0] + inc * 2.0 * directions[0], points[2] + inc * 2.0 * directions[3]),
    std::make_pair(points[0] + inc * 2.0 * directions[2], points[2] + inc * 2.0 * directions[1]),
    std::make_pair(points[0] + inc * 3.0 * directions[0], points[2] + inc * 3.0 * directions[3]),
    std::make_pair(points[0] + inc * 3.0 * directions[2], points[2] + inc * 3.0 * directions[1]),
    // std::make_pair(points[0] + inc * 4.0 * directions[0], points[2] + inc * 4.0 * directions[1]),
    // std::make_pair(points[0] + inc * 4.0 * directions[0], points[2] + inc * 4.0 * directions[1]),
    // std::make_pair(points[0] + inc * 5.0 * directions[2], points[2] + inc * 5.0 * directions[3]),
    // std::make_pair(points[0] + inc * 5.0 * directions[2], points[2] + inc * 5.0 * directions[3]),
    // std::make_pair(points[0] + inc * 6.0 * directions[0], points[2] + inc * 6.0 * directions[1]),
    // std::make_pair(points[0] + inc * 6.0 * directions[2], points[2] + inc * 6.0 * directions[3])
  };

  for (auto const& line : lines)
  {
    if (!isnan(line.first.x) && !isnan(line.first.y) && !isnan(line.second.x) && !isnan(line.second.y))
    {
      renderPreSdlConvertedLineAsRendererIs(line.first, line.second);
    }
  }

} 


void renderTransparentObject(const pce::RigidObject& rigid_object, const std::vector<int>& color)
{
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  for (auto const& [edge_id, vertex_ids] : rigid_object.edges)
  {
    renderLineAsRendererIs(rigid_object.vertex_pixels.at(vertex_ids.first) * pce3d::Core3D::ORDINARY_ZOOM_INDEX_,
                           rigid_object.vertex_pixels.at(vertex_ids.second) * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
    // renderLineAsRendererIs(rigid_object.camera_transformed_vertices.at(vertex_ids.first),
                          //  rigid_object.camera_transformed_vertices.at(vertex_ids.second));
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}



void renderEllipse(
    const glm::dvec2& a_focus
  , const glm::dvec2& b_focus
  , const double semi_major_axis
  , const int num_sides
  , const std::vector<int> color
)
{
  const glm::dvec2 center = (a_focus + b_focus) / 2.0;
  const std::vector<glm::ivec2> pixels = pce::raster::getEllipseRasterPoints(
    center,
    a_focus,
    b_focus,
    semi_major_axis,
    num_sides);
  
  setRendererColor(color);
  for (size_t i = 1; i < pixels.size(); ++i)
  {
    if (i == pixels.size()/2) { continue; }
    pce::render::renderLineAsRendererIs(glm::dvec2(pixels[i-1].x, pixels[i-1].y),
                                        glm::dvec2(pixels[i].x, pixels[i].y));
  }
  setRendererColor({0, 0, 0, 255});


}
 


void renderUnOrdinaryZoomedLine(const glm::dvec2& a, const glm::dvec2& b, std::vector<int> color)
{
  const glm::dvec2 sdl_pixel_a = pce::convert::convertCartesianCoordinatesToSDL(a * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  const glm::dvec2 sdl_pixel_b = pce::convert::convertCartesianCoordinatesToSDL(b * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  SDL_RenderDrawLine(Simulation::renderer, sdl_pixel_a.x, sdl_pixel_a.y,
                                           sdl_pixel_b.x, sdl_pixel_b.y);
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255); 
}


} 
}


#endif /* render_functions_cpp */
