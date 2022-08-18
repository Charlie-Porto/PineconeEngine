#ifndef triangle_raster_functions_cpp
#define triangle_raster_functions_cpp


#include "../triangle_raster_functions.hpp"


namespace pce3d {
namespace raster {

const double zoom_ratio = pce3d::Core3D::ORDINARY_ZOOM_INDEX_;
const double PI = 3.14159265;

void rasterizeAndRenderTriangle(const maths::Triangle& tri, const std::vector<int>& color) {
  auto triangle = tri;
  triangle.A *= zoom_ratio;
  triangle.B *= zoom_ratio;
  triangle.C *= zoom_ratio;
  sortTriangleVertices(triangle);
  
  pce::render::setRendererColor(color);
  // pce::render::setRendererColor({200, 0, 0, 255});
  const double size_tolerance = 0.01;
  if (abs(triangle.A.y - triangle.B.y) < (size_tolerance * zoom_ratio)
   && abs(triangle.A.y - triangle.C.y) < (size_tolerance * zoom_ratio)
  ) { 
    // pce::quickdraw::drawLine(triangle.A, triangle.B, color);
    // pce::quickdraw::drawLine(triangle.C, triangle.B, color);
    // pce::quickdraw::drawLine(triangle.A, triangle.C, color);
  } else {

    pce::render::renderLineAsRendererIs(triangle.A, triangle.B);
    pce::render::renderLineAsRendererIs(triangle.A, triangle.C);
    pce::render::renderLineAsRendererIs(triangle.B, triangle.C);

    rasterizeAndRenderTriangleTopHalf(triangle, color);
    if ( abs(triangle.B.y - triangle.C.y) > size_tolerance * zoom_ratio ) {
      // pce::render::setRendererColor({0, 200, 0, 255});
      rasterizeAndRenderTriangleLowerHalf(triangle, color);
    }
  }
  pce::render::setRendererColor({0, 0, 0, 255});
}

void sortTriangleVertices(maths::Triangle& triangle) {
  std::vector<glm::dvec2> sorted_points = {triangle.A, triangle.B, triangle.C};

  while (true) {
    if (sorted_points[0].y >= sorted_points[1].y && sorted_points[1].y >= sorted_points[2].y){
      break;
    } else {
      for (int i = 0; i < 2; ++i) { 
        if (sorted_points[i].y < sorted_points[i+1].y) {
          glm::dvec2 temp = sorted_points[i];
          sorted_points[i] = sorted_points[i+1];
          sorted_points[i+1] = temp;
        }
      }
    }
  }

  triangle.A = sorted_points[0];
  triangle.B = sorted_points[1];
  triangle.C = sorted_points[2];
}

const double angle_threshold_for_render = 1.58;

void rasterizeAndRenderTriangleTopHalf(const maths::Triangle& triangle, const std::vector<int>& color) {

  const glm::dvec2 height_vector = glm::dvec2(0, 1);
  const glm::dvec2 long_side = triangle.A - triangle.C; 
  const glm::dvec2 short_side = triangle.A - triangle.B; 
  
  /* calculate angles of sides relative to height vector */
  const double long_angle = acos(glm::dot(long_side, height_vector) 
                            / (sqrt(glm::dot(long_side, long_side)) 
                            * sqrt(glm::dot(height_vector, height_vector))));
  const double short_angle = acos(glm::dot(short_side, height_vector) 
                            / (sqrt(glm::dot(short_side, short_side)) 
                            * sqrt(glm::dot(height_vector, height_vector))));

  const double long_angle_sign = pce::math::sign(triangle.C.x - triangle.A.x);
  const double short_angle_sign = pce::math::sign(triangle.B.x - triangle.A.x);
  
  double long_side_x_crawl_distance = std::min(tan(long_angle), sqrt(glm::dot(triangle.A - triangle.C, triangle.A - triangle.C)));
  double short_side_x_crawl_distance = std::min(tan(short_angle), sqrt(glm::dot(triangle.A - triangle.B, triangle.A - triangle.B)));

  double crawl_number = 0;
  if (long_angle > angle_threshold_for_render) { long_side_x_crawl_distance = 0.0; }
  if (short_angle > angle_threshold_for_render) { short_side_x_crawl_distance = 0.0; }
  for (int i = triangle.A.y; i > triangle.B.y; --i) {
    auto const long_side_crawl_point 
        = glm::dvec2(
            triangle.A.x + std::min(long_side_x_crawl_distance * crawl_number, abs(triangle.C.x - triangle.A.x)) * long_angle_sign, 
            i);
    auto const short_side_crawl_point 
        = glm::dvec2(
            triangle.A.x + std::min(short_side_x_crawl_distance * crawl_number, abs(triangle.B.x - triangle.A.x)) * short_angle_sign, 
            i);

    pce::render::renderLineAsRendererIs(long_side_crawl_point, short_side_crawl_point);
    ++crawl_number;
  }

  // const glm::dvec2 final_long_crawl = glm::dvec2(triangle.A.x + long_side_x_crawl_distance * std::max(1.0, crawl_number) * long_angle_sign, triangle.B.y);
  // pce::render::renderLineAsRendererIs(triangle.B, final_long_crawl);

}

void rasterizeAndRenderTriangleLowerHalf(const maths::Triangle& triangle, const std::vector<int>& color) {
  const glm::dvec2 height_vector = glm::dvec2(0, -1);
  const glm::dvec2 long_side = triangle.C - triangle.A; 
  const glm::dvec2 short_side = triangle.C - triangle.B; 
  
  /* calculate angles of sides relative to height vector */
  const double long_angle = acos(glm::dot(long_side, height_vector) 
                            / (sqrt(glm::dot(long_side, long_side)) 
                            * sqrt(glm::dot(height_vector, height_vector))));
  const double short_angle = acos(glm::dot(short_side, height_vector) 
                            / (sqrt(glm::dot(short_side, short_side)) 
                            * sqrt(glm::dot(height_vector, height_vector))));

  const double long_angle_sign = pce::math::sign(triangle.A.x - triangle.C.x);
  const double short_angle_sign = pce::math::sign(triangle.B.x - triangle.C.x);
  double long_side_x_crawl_distance = std::min(tan(long_angle), sqrt(glm::dot(triangle.C - triangle.A, triangle.C - triangle.A)));
  double short_side_x_crawl_distance = std::min(tan(short_angle), sqrt(glm::dot(triangle.C - triangle.B, triangle.C - triangle.B)));

  double crawl_number = 0;
  if (long_angle > angle_threshold_for_render) { long_side_x_crawl_distance = 0.0; }
  if (short_angle > angle_threshold_for_render) { short_side_x_crawl_distance = 0.0; }
  for (int i = triangle.C.y; i < triangle.B.y; ++i) {
    auto const long_side_crawl_point 
        = glm::dvec2(
            triangle.C.x + std::min(long_side_x_crawl_distance * crawl_number, abs(triangle.A.x - triangle.C.x)) * long_angle_sign, 
            i);
    auto const short_side_crawl_point 
        = glm::dvec2(
            triangle.C.x + std::min(short_side_x_crawl_distance * crawl_number, abs(triangle.B.x - triangle.C.x)) * short_angle_sign, 
            i);
    pce::render::renderLineAsRendererIs(long_side_crawl_point, short_side_crawl_point);
    ++crawl_number;
  }
}
  




}
}

#endif /* triangle_raster_functions_cpp */

