#ifndef main_cpp
#define main_cpp

#include <pceSDL/pceSDL.hpp>
#include <pcecs/pcecs.hpp>
#include <pce3d/pce3d.hpp>


#include <pce3d/entity_forging/square_pyramid_forging.hpp>
#include <pce3d/entity_forging/triangle_pyramid_forging.hpp>


ControlPanel control;
pce3d::DevRenderSystem dev_render_system;


void makeTentLand() {

  int count = 500;
  const double max_x = 900.0;
  const double max_z = 900.0;
  const double max_h = 20.0;
  const double max_w = 30.0;

  for (int i = 0; i < count; ++i) {
    const double height = pce3d::random::getRandomDoubleBetweenDoubles(2.0, max_h);
    const double width = pce3d::random::getRandomDoubleBetweenDoubles(2.0, max_w);
    const double xpos = pce3d::random::getRandomDoubleBetweenZeroAndDouble(max_x);
    const double ypos = height/2.0 - 4.0;
    const double zpos = -pce3d::random::getRandomDoubleBetweenZeroAndDouble(max_z);
    const std::vector<int> color = pce3d::random::getRandomColor();

    const int coin = pce3d::random::getCoinFlipResult();
    Entity a;
    switch(coin) {
      case 0:
        a = pce3d::forge::forgeSquarePyramidEntity(height, width, glm::dvec3(xpos, ypos, zpos), {1.0, 0, 0, 0}, color);
        break;
      case 1:
        a = pce3d::forge::forgeTrianglePyramidEntity(height, width, glm::dvec3(xpos, ypos, zpos), {1.0, 0, 0, 0}, color);
        break;
      default:
        break;
    }
  }
}



int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  control.Init();
  auto core_3d = new pce3d::Core3D();

  makeTentLand();

  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();
    core_3d->UpdateCore3D();
    core_manager->DoCorePostUpdate();
  }






  delete core_3d;
  delete core_manager;

  return 0;
}







#endif /* main_cpp */
