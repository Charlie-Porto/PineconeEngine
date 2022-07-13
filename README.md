# Pinecone Engine
an engine and library for 3D graphics (and my learn-how-to-make-a-library attempt)
<br/>

## Features
- simple event loop setup
- entity component system
- a handful of pre-implemented components and systems available for use in projects as well as boilerplates for custom components and systems
- access to functionalities of the [SDL2](https://github.com/libsdl-org/SDL) and [glm](https://github.com/g-truc/glm) libraries
<br/>


## Basic Example
```c++
#include <pce/core.hpp>
#include <pce/tools/simple_framerate_timer.cpp>


ControlPanel control;
int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  simple_framerate_timer timer = simple_framerate_timer();


  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();

    timer.update_timer(int(core_manager->getFrameStartTime()));

    core_manager->DoCorePostUpdate();
  }


  delete core_manager;
  delete ecs_manager;
  return 0;
}

```
<br/>


## Requirements
- C++17 compiler (any should work)
- As the library is still under development, the [SDL2](https://github.com/libsdl-org/SDL) and [glm](https://github.com/g-truc/glm) libraries will need to be obtained and built separately
  - Note: one of the primary goals of this project is to create a CMakeLists.txt file that can snag and build these dependencies as a side-effect of building the library. This functionality should be available soon.
<br/>


## Compiling a project that uses the **Pinecone Engine**
- to compile your project, include the path to **pce.cpp** in the compilation source file inputs. I can compile like this using gcc:
```zsh
g++ src/main.cpp -o main.o -std=c++17 \
    /Path_to_PinceconeEngine/pce/pce.cpp \
    -lSDL2-2.0.0 \ # sdl include
    -lSDL2_image-2.0.0 \ #sdl image include
```
<br/>


## Acknowledgements
- Thank you to Austin Morland for his Entity Component System design accompanied by fantastic documentation which can be found [here](https://austinmorlan.com/posts/entity_component_system/)
- Thank you to Lazy Foo' Productions for their SDL2 tutorial which can be found [here](https://lazyfoo.net/tutorials/SDL/)
- Thank you to Jeremiah for writing this [article](https://www.3dgep.com/understanding-quaternions/) on quaternions which I found to be the best explanation around