#ifndef main_cpp
#define main_cpp

#include <pceSDL/pceSDL.hpp>
#include <pcecs/pcecs.hpp>
#include <pce3d/pce3d.hpp>

#include <pce3d/entity_forging/rectangular_prism_forging.hpp>

ControlPanel control;

void createRandomRectPrism() {
  double w = pce3d::random::getRandomDoubleBetweenDoubles(1.0, 100.0);
  double h = pce3d::random::getRandomDoubleBetweenDoubles(1.0, 100.0);
  double l = pce3d::random::getRandomDoubleBetweenDoubles(1.0, 100.0);

  double x = pce3d::random::getRandomDoubleBetweenDoubles(2, 1000);
  double y = pce3d::random::getRandomDoubleBetweenDoubles(1, 300);
  double z = -pce3d::random::getRandomDoubleBetweenDoubles(30, 1000);

  std::vector<int> color = pce3d::random::getRandomColor();

  auto a = pce3d::forge::forgeRectPrismEntity(w, h, l, glm::dvec3(x, y, z), {1.0, 0, 0, 0}, color);
  /* 
  does nothing with a -- fine because it's been accounted for by the control panel 
  (could disable compiler flag for now) 
  */
}


int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  control.Init();
  auto core_3d = new pce3d::Core3D();

  core_3d->PrepareForAllSystemsGo();
  for (int i = 0; i < 50; ++i) {
    createRandomRectPrism();
  }

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
