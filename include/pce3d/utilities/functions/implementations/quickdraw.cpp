#ifndef quickdraw_cpp
#define quickdraw_cpp

#include "../quickdraw.hpp"


namespace pce {
namespace quickdraw {

void drawCircle(const glm::dvec2& center_point, double radius, const std::vector<int>& color) {
  const glm::vec2 sdl_transform = pce::convert::convertCartesianCoordinatesToSDL(center_point * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  render::renderCircle(sdl_transform.x, sdl_transform.y, int(radius * pce3d::Core3D::ORDINARY_ZOOM_INDEX_), color);
}


void drawFilledCircle(const glm::dvec2& center_point, double radius, const std::vector<int>& color) {
  const glm::vec2 ncenter_point = pce::convert::convertCartesianCoordinatesToSDL(center_point * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  pce::render::renderFilledCircle(ncenter_point.x, ncenter_point.y, radius, color);
}

void drawFilledCircleClean(const glm::dvec2& center_point, double radius, const std::vector<int>& color) {
  const glm::vec2 ncenter_point = pce::convert::convertCartesianCoordinatesToSDL(center_point * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  const std::unordered_map<glm::dvec2, glm::dvec2> points = pce::raster::getCircleOutlinePixelPairs(ncenter_point.x, ncenter_point.y, int(radius));
  
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  for (auto const& [A, B] : points) {
    SDL_RenderDrawLine(Simulation::renderer, int(A.x), int(A.y), int(B.x), int(B.y));
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}


void drawSetOfEdges(const std::vector<std::pair<glm::dvec2, glm::dvec2>>& edges, 
                    const std::vector<int>& color) {
   
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);

  for (auto const& edge : edges) {
    const glm::dvec2 sdl_vertex_a = pce::convert::convertCartesianCoordinatesToSDL(edge.first * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
    const glm::dvec2 sdl_vertex_b = pce::convert::convertCartesianCoordinatesToSDL(edge.second * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);

    SDL_RenderDrawLine(
      Simulation::renderer,
      sdl_vertex_a.x, sdl_vertex_a.y,
      sdl_vertex_b.x, sdl_vertex_b.y
    );                                           

  }

  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);

}

   
void drawFilledRect(const glm::dvec2& top_L_corner, const glm::dvec2& lower_R_corner, 
                    const std::vector<int>& color) {
  const double x_left = top_L_corner.x;
  const double x_right = lower_R_corner.x;
  double y_current = top_L_corner.y;

  
  SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  while (y_current >= lower_R_corner.y) {
    const glm::dvec2 sdl_vertex_a = pce::convert::convertCartesianCoordinatesToSDL(glm::dvec2(x_left, y_current) * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
    const glm::dvec2 sdl_vertex_b = pce::convert::convertCartesianCoordinatesToSDL(glm::dvec2(x_right, y_current) * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
    SDL_RenderDrawLine(
      Simulation::renderer,
      sdl_vertex_a.x, sdl_vertex_a.y,
      sdl_vertex_b.x, sdl_vertex_b.y
    );                                           
  }
  SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}



void drawFilledQuadrilateral(const pce3d::maths::Quadrilateral& q,
                             const std::vector<int>& color) {
  auto tri_a = pce3d::maths::Triangle{.A=q.A, .B=q.B, .C=q.C};
  auto tri_b = pce3d::maths::Triangle{.A=q.A, .B=q.D, .C=q.C};
  auto tri_c = pce3d::maths::Triangle{.A=q.B, .B=q.A, .C=q.D};
  auto tri_d = pce3d::maths::Triangle{.A=q.B, .B=q.C, .C=q.D};

  pce3d::raster::rasterizeAndRenderTriangle(tri_a, color);
  pce3d::raster::rasterizeAndRenderTriangle(tri_b, color);
  pce3d::raster::rasterizeAndRenderTriangle(tri_c, color);
  pce3d::raster::rasterizeAndRenderTriangle(tri_d, color);


  // SDL_SetRenderDrawColor(Simulation::renderer, color[0], color[1], color[2], color[3]);
  // const glm::dvec2 a = pce::convert::convertCartesianCoordinatesToSDL(q.A * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  // const glm::dvec2 b = pce::convert::convertCartesianCoordinatesToSDL(q.B * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  // const glm::dvec2 c = pce::convert::convertCartesianCoordinatesToSDL(q.C * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);
  // const glm::dvec2 d = pce::convert::convertCartesianCoordinatesToSDL(q.D * pce3d::Core3D::ORDINARY_ZOOM_INDEX_);

  /* draw diagonals */
  // std::vector<glm::dvec2> points = {a, b, c, d}; 
  // pce::render::renderQuadrilateralDiagonals(points, color);
  
  // SDL_SetRenderDrawColor(Simulation::renderer, 0, 0, 0, 255);
}


void drawFilledTriangle(const pce3d::maths::Triangle& triangle,
                        const std::vector<int>& color) {
  pce3d::raster::rasterizeAndRenderTriangle(triangle, color);
}

}
}




#endif /* quickdraw_cpp */
