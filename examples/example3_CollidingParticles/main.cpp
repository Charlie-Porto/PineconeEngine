#ifndef main_cpp
#define main_cpp

#include <pceSDL/pceSDL.hpp>
#include <pcecs/pcecs.hpp>
#include <pce3d/pce3d.hpp>

#include <pce3d/entity_forging/sphere_forging.hpp>
#include <kelp_randomness.hpp>

void generateParticles(int n) {
  for (int i = 0; i < n; ++i) {
    const double xpos = kelp::random::getRandomSignedDoubleBetweenDoubles(1, 200);
    const double ypos = kelp::random::getRandomSignedDoubleBetweenDoubles(1, 200);
    const double zpos = kelp::random::getRandomDoubleBetweenDoubles(30, 35);

    const double speed = kelp::random::getRandomDoubleBetweenZeroAndDouble(1.0)/10.0;
    const glm::dvec3 dir = glm::dvec3(-xpos, -ypos, zpos);
    const glm::dvec3 vel = dir * speed; 

    auto a = pce3d::forge::forgeSphereEntity(1.0, glm::dvec3(xpos, ypos, zpos), {20, 230, 150, 255});
     
    control.AddComponent(a, pce::Motion{
    .speed = speed,
    .direction = dir,
    .velocity = vel,
    .rotational_speed = 0.0,
    .rotational_axis = glm::dvec3(0, 0, 0),
    .duration = 0.0,
    .previous_resting_position = glm::dvec3(xpos, ypos, zpos)
  });

  }
}


ControlPanel control;
int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  control.Init();
  auto core_3d = new pce3d::Core3D();
  generateParticles(300);
  
  core_3d->PrepareForAllSystemsGo();
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
