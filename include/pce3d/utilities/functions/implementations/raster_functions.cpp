#ifndef raster_functions_cpp
#define raster_functions_cpp

#include "../raster_functions.hpp"

namespace pce {
namespace raster {

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
    points_list[glm::dvec2(xc-x, yc-y)] = glm::dvec2(xc+x, yc-y);
    points_list[glm::dvec2(xc-y, yc-x)] = glm::dvec2(xc+y, yc-x);
    points_list[glm::dvec2(xc-y, yc+x)] = glm::dvec2(xc+y, yc+x);
  }
  return points_list;
}


}

}



#endif /* raster_functions_cpp */
