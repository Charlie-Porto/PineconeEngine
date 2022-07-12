# Pinecone Engine
an engine + library for 3D graphics (and my learn-how-to-make-a-library attempt)

## Features
- simple event loop setup
- entity component system
- a handful of pre-implemented components and systems available for used in projects as well as boilerplates for custom components and systems
- access to functionalities of the [SDL2](https://github.com/libsdl-org/SDL) and [glm](https://github.com/g-truc/glm) libraries



## Basic Example
```c++
#include <pce/CoreManager.h>
#include <pce/EcsManager.h>
#include <pce/tools/simple_framerate_timer.cpp>


ControlPanel control;
int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  auto ecs_manager = new pce::EcsManager();

  /* initialize timer */  
  simple_framerate_timer timer = simple_framerate_timer();

  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();

    /* update timer */
    timer.update_timer(int(core_manager->getFrameStartTime()));


    core_manager->DoCorePostUpdate();
  }


  delete core_manager;
  delete ecs_manager;
  return 0;
}

```



