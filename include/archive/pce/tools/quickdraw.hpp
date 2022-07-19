#ifndef quickdraw_hpp
#define quickdraw_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to render simple objects easily 
-----------------------------------------------------------------*/

#include <vector>
#include <glm/vec2.hpp>

namespace pce {
namespace quickdraw {

void drawListOfPixels(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color, double zoom_ratio);

void drawSinglePixel(const glm::dvec2& pixel, const std::vector<int>& color, double zoom_ratio);

void drawLine(const glm::dvec2& point_a, const glm::dvec2& point_b, const std::vector<int>& color, double zoom_ratio);

void drawCircle(const glm::dvec2& center_point, double radius, const std::vector<int>& color, double zoom_ratio);

void drawSetOfEdges(const std::vector<std::pair<glm::dvec2, glm::dvec2>>& edges, 
                    const std::vector<int>& color, double zoom_ratio);
   
}
}







#endif /* quickdraw_hpp */
