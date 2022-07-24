#ifndef triangle_raster_functions_cpp
#define triangle_raster_functions_cpp

#include <cmath>
#include <glm/geometric.hpp>
#include "../triangle_raster_functions.hpp"
#include "../../../maths/functions/sign.hpp"
#include "../quickdraw.hpp"
#include <vezprint.cpp>


namespace pce3d {
namespace raster {

const double zoom_ratio = 10.0;


void rasterizeAndRenderTriangle(maths::Triangle& triangle, const std::vector<int>& color) {
  triangle.A *= zoom_ratio;
  triangle.B *= zoom_ratio;
  triangle.C *= zoom_ratio;
  sortTriangleVertices(triangle);
  
  if (triangle.A == triangle.B) { --triangle.B; }

  rasterizeAndRenderTriangleTopHalf(triangle, color);
  if (triangle.B.y != triangle.C.y) {
    rasterizeAndRenderTriangleLowerHalf(triangle, color);
  }
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


void rasterizeAndRenderTriangleTopHalf(maths::Triangle& triangle, const std::vector<int>& color) {

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
  
  double long_side_x_crawl_distance = tan(long_angle);
  double short_side_x_crawl_distance = tan(short_angle);
  double crawl_number = 0;

  for (int i = triangle.A.y; i > triangle.B.y; --i) {
    auto const long_side_crawl_point 
        = glm::dvec2(triangle.A.x + long_side_x_crawl_distance * crawl_number * long_angle_sign, i);
    auto const short_side_crawl_point 
        = glm::dvec2(triangle.A.x + short_side_x_crawl_distance * crawl_number * short_angle_sign, i);
    pce::quickdraw::drawLine(long_side_crawl_point, short_side_crawl_point, color, 1.0);
    ++crawl_number;
  }
  const glm::dvec2 final_long_crawl = glm::dvec2(triangle.A.x + long_side_x_crawl_distance * crawl_number * long_angle_sign, triangle.B.y);
  pce::quickdraw::drawLine(triangle.B, final_long_crawl, color, 1.0);


}

void rasterizeAndRenderTriangleLowerHalf(maths::Triangle& triangle, const std::vector<int>& color) {
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

  double long_side_x_crawl_distance = tan(long_angle);
  double short_side_x_crawl_distance = tan(short_angle);
  double crawl_number = 0;

  for (int i = triangle.C.y; i < triangle.B.y; ++i) {
    auto const long_side_crawl_point 
        = glm::dvec2(triangle.C.x + long_side_x_crawl_distance * crawl_number * long_angle_sign, i);
    auto const short_side_crawl_point 
        = glm::dvec2(triangle.C.x + short_side_x_crawl_distance * crawl_number * short_angle_sign, i);
    pce::quickdraw::drawLine(long_side_crawl_point, short_side_crawl_point, color, 1.0);
    ++crawl_number;
  }
}
  




}
}

#endif /* triangle_raster_functions_cpp */

