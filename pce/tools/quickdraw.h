#ifndef quickdraw_h
#define quickdraw_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to render simple objects easily 
-----------------------------------------------------------------*/

#include <vector>
#include <glm/vec2.hpp>

namespace pce {
namespace quickdraw {

void drawListOfPixels(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color);
void drawSinglePixel(const glm::dvec2& pixel, const std::vector<int>& color);
// void drawCircleAtVec2(const glm::dvec2 center, double radius, const std::vector<int>& color);
   
}
}







#endif /* quickdraw_h */
