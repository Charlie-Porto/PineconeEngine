

#include <pce/core.hpp>
#include <pce/tools/simple_framerate_timer.hpp>

int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  simple_framerate_timer timer = simple_framerate_timer();


  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();

    timer.update_timer(int(core_manager->getFrameStartTime()));

    core_manager->DoCorePostUpdate();
  }


  delete core_manager;
  return 0;
}


