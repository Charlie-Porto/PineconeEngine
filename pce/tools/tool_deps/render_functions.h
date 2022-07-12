#ifndef render_functions_h
#define render_functions_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to render things
-----------------------------------------------------------------*/

#include <vector>
#include <glm/vec2.hpp>

/* include Simulation.h so that the renderer can be accessed */
#include "../../core/Simulation.h"

namespace pce {
namespace render {

void renderPixelList(const std::vector<glm::dvec2>& pixels, const std::vector<int>& color);
void renderPixel(const glm::dvec2& pixel, const std::vector<int>& color);

}
}




#endif /* render_functions_h */
