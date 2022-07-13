
#include <pce/core.hpp>
#include <pce/tools/quickdraw.hpp>

ControlPanel control;
int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();

  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();
    /* draw a white pixel in the center of the screen */
    pce::quickdraw::drawSinglePixel(glm::dvec2(0, 0), {255, 255, 255, 255});

    core_manager->DoCorePostUpdate();
  }


  delete core_manager;
  return 0;
}






