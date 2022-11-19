#ifndef raster_functions_cpp
#define raster_functions_cpp

#include "../raster_functions.hpp"

namespace pce {
namespace raster {

const double PI = 3.14159265;

/* for Circle Drawing */
std::vector<glm::dvec2> getCircleOctet(int xc, int yc, int x, int y) {
  std::vector<glm::dvec2> points;
  points.push_back(glm::dvec2(xc+x, yc+y));
  points.push_back(glm::dvec2(xc-x, yc+y));
  points.push_back(glm::dvec2(xc+x, yc-y));
  points.push_back(glm::dvec2(xc-x, yc-y));
  points.push_back(glm::dvec2(xc+y, yc+x));
  points.push_back(glm::dvec2(xc-y, yc+x));
  points.push_back(glm::dvec2(xc+y, yc-x));
  points.push_back(glm::dvec2(xc-y, yc-x));
  return points;
}

std::unordered_map<glm::dvec2, glm::dvec2> getCircleOctetPairs(int xc, int yc, int x, int y) {
  std::unordered_map<glm::dvec2, glm::dvec2> points_list = {
    {glm::dvec2(xc-x, yc+y), glm::dvec2(xc+x, yc+y)},
    {glm::dvec2(xc-x, yc-y), glm::dvec2(xc+x, yc-y)},
    {glm::dvec2(xc-y, yc-x), glm::dvec2(xc+y, yc-x)},
    {glm::dvec2(xc-y, yc+x), glm::dvec2(xc+y, yc+x)}
  };
  return points_list;
}


std::vector<glm::dvec2> getCircleRasterizationPoints(int xc, int yc, int r) {
  int x = 0;
  int y = round(r)+1;
  // std::cout << "rounded circle radius: " << y << '\n';
  int d = 3 - 2 * r;
  std::vector<glm::dvec2> points_list = getCircleOctet(xc, yc, x, y);
  while (y >= x) {
    x++;
    if (d > 0) {
      y--;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    std::vector<glm::dvec2> new_points = {
      glm::dvec2(xc+x, yc+y),
      glm::dvec2(xc-x, yc+y),
      glm::dvec2(xc+x, yc-y),
      glm::dvec2(xc-x, yc-y),
      glm::dvec2(xc+y, yc+x),
      glm::dvec2(xc-y, yc+x),
      glm::dvec2(xc+y, yc-x),
      glm::dvec2(xc-y, yc-x)
    };
    points_list.insert(points_list.end(), new_points.begin(), new_points.end());
  }
  return points_list;
}

std::unordered_map<glm::dvec2, glm::dvec2> getCircleOutlinePixelPairs(int xc, int yc, int r) {
  int x = 0;
  int y = r;
  int d = 3 - 2 * r;
  std::unordered_map<glm::dvec2, glm::dvec2> points_list = getCircleOctetPairs(xc, yc, x, y);
  while (y >= x) {
    x++;
    if (d > 0) {
      y--;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    points_list[glm::dvec2(xc-x, yc+y)] = glm::dvec2(xc+x, yc+y);
    
    points_list[glm::dvec2(xc-y, yc+x)] = glm::dvec2(xc+y, yc+x);
  }
  return points_list;
}



std::vector<glm::ivec2> getEllipseRasterPoints(
    const glm::dvec2& center
  , const glm::dvec2& a_focus
  , const glm::dvec2& b_focus
  , const double semi_major_axis
  , const int num_sides
)
{
  std::vector<glm::ivec2> pixels = {};
  std::vector<glm::ivec2> pixels_other_side = {};

  glm::dvec2 ellipse_point = glm::normalize(b_focus - a_focus) * semi_major_axis;
  const double b_focus_distance = pce3d::maths::calcMagV2((b_focus - center) - ellipse_point);
  // std::cout << "b_focus_distance: " << b_focus_distance << '\n';
  // const double l = b_focus_distance + pce3d::maths::calcMagV2(b_focus - a_focus) + b_focus_distance;
  // std::cout << "l: "  << l << '\n';

  // rotate ellipse point  
  // const glm::dvec2 a_point = ellipse_point;
  // const glm::dvec2 b_point = glm::dvec2(calcMagV2(b_focus - center), 0.0);
  // const std::complex<double> rotator_complex_num = pce3d::maths::calculateRotationComplexNumGivenVec2s(
    // b_point, a_point);
  const glm::dvec2 unit_b_focus = glm::dvec2(1, 0);
  const glm::dvec2 rotated_b_focus = unit_b_focus * pce3d::maths::calcMagV2(b_focus - center);
  const std::complex<double> rotator_complex_num = pce3d::maths::calculateRotationComplexNumGivenVec2s(
    rotated_b_focus, (b_focus - center));
  
  // std::cout << "rotator_complex_num" << rotator_complex_num << '\n';
  
  const double a = pce3d::maths::calculateDistanceBetweenVectors(center, center + ellipse_point);
  const double c = pce3d::maths::calculateDistanceBetweenVectors(center, b_focus);
  const double b = sqrt(a*a - c*c);
  
  const double incrememtal_angle = 360.0 / double(num_sides);
  double current_angle = 0.0;
  
  // get points
  for (double i = 0.0; int(i) <= num_sides/2.0 + 1.0; ++i)
  {
    current_angle = incrememtal_angle * i;
    // const double aim_circle_point_x = (unit_b_focus.x - center.x) * cos(current_angle / 180.0 * PI) + center.x;
    // std::cout << "current angle: " << current_angle << '\n';
    const double aim_circle_point_x = unit_b_focus.x * cos(current_angle / 180.0 * PI);
    double x = aim_circle_point_x * semi_major_axis;
    const double x_sq_over_a_sq = (x*x) / (a*a);
    const double eq_right_side = (1.0 - x_sq_over_a_sq) * (b*b);
    double y = sqrt(eq_right_side);
    if (isnan(x)) { x = 0.0; }
    if (isnan(y)) { y = 0.0; }
    // std::cout << "x: " << x << '\n';
    // std::cout << "y: " << y << '\n';
    // const double aim_circle_point_y = sqrt(1.0 - pow(aim_circle_point_x, 2.0)) + center.y;
    // const glm::dvec2 aim_circle_point = 50.0 * glm::dvec2(aim_circle_point_x, aim_circle_point_y);
    // glm::ivec2 aim_circle_point_int = pce::convert::convertIntCartesianCoordinatesToIntSDL(
      // glm::ivec2(int(aim_circle_point.x), int(aim_circle_point.y)));
    // pixels.push_back(aim_circle_point_int);

    // un-reverse points
    glm::dvec2 ellipse_pixel_pos_unrot = glm::dvec2(x, y);
    glm::dvec2 ellipse_pixel_neg_unrot = glm::dvec2(x, -y);

    const double pos_dist = pce3d::maths::calcMagV2(ellipse_pixel_pos_unrot);
    const double neg_dist = pce3d::maths::calcMagV2(ellipse_pixel_neg_unrot);

    ellipse_pixel_pos_unrot = glm::normalize(pce3d::maths::rotateVec2ByComplexNumber(ellipse_pixel_pos_unrot, rotator_complex_num))
                            * pos_dist
                            + center;
    ellipse_pixel_neg_unrot = glm::normalize(pce3d::maths::rotateVec2ByComplexNumber(ellipse_pixel_neg_unrot, rotator_complex_num))
                            * neg_dist
                            + center;

    // glm::ivec2 ellipse_pixel_pos = pce::convert::convertIntCartesianCoordinatesToIntSDL(
      // glm::ivec2(int(ellipse_pixel_pos_unrot.x), int(ellipse_pixel_pos_unrot.y)));
    // glm::ivec2 ellipse_pixel_neg = pce::convert::convertIntCartesianCoordinatesToIntSDL(
      // glm::ivec2(int(ellipse_pixel_neg_unrot.x), int(ellipse_pixel_neg_unrot.y)));
    glm::ivec2 ellipse_pixel_pos = glm::ivec2(int(ellipse_pixel_pos_unrot.x), int(ellipse_pixel_pos_unrot.y));
    glm::ivec2 ellipse_pixel_neg = glm::ivec2(int(ellipse_pixel_neg_unrot.x), int(ellipse_pixel_neg_unrot.y));

    // std::cout << "ellipse_pixel_pos: " << ellipse_pixel_pos.x << ", " << ellipse_pixel_pos.y << '\n';
    // std::cout << "ellipse_pixel_neg: " << ellipse_pixel_neg.x << ", " << ellipse_pixel_neg.y << '\n';

    pixels.push_back(ellipse_pixel_pos);
    pixels_other_side.push_back(ellipse_pixel_neg);
  }
  std::vector<glm::ivec2> rpixels = pixels; 
  rpixels.insert(rpixels.end(), pixels_other_side.begin(), pixels_other_side.end());
  return rpixels;
}



}

}



#endif /* raster_functions_cpp */
