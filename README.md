# Pinecone Engine
a three-unit library for 3D graphics and games (and my introduction to library authoring)
- unit 1: SDL2 event loop wrapper 
- unit 2: entity component system 
- unit 3: 3D rendering tools
<br/>

## Quick Setup Example
the code below pulls up a basic black screen that loops until quit by the user
```c++
#include <pceSDL/pceSDL.hpp>
#include <pcecs/pcecs.hpp>
#include <pce3d/pce3d.hpp>

ControlPanel control;
int main(int argc, const char* argv[]) {

  auto core_manager = new pce::CoreManager();
  control.Init();
  auto core_3d = new pce3d::Core3D();

  while (core_manager->Running()) {
    core_manager->DoCorePreUpdate();
    core_3d->UpdateCore3D();
    core_manager->DoCorePostUpdate();
  }

  delete core_3d;
  delete core_manager;

  return 0;
}
```
<br/>


## Requirements
- C++17 compiler (any should work)
- The [SDL2](https://github.com/libsdl-org/SDL) and [glm](https://github.com/g-truc/glm) libraries will need to be obtained and built separately. Fortunately, the SDL library contains robust build instructions, and the glm library is header-only.
<br/>


## Compiling Projects that use the PineconeEngine
- proper make support (cmake) is currently in the works. for now, please make sure to add the flags below to the compilation command. (note that text inside brackets <> should be replaced by you based on your config)
```
  -I/<path_to_PineconeEngine>/include \
    /<path_to_PineconeEngine>/include/pceSDL/pceSDL.cpp \
    /<path_to_PineconeEngine>/include/pce3d/pce3d.cpp \
  -lSDL2-2.0.0 \
  -lSDL2_image-2.0.0 \
```


<br/>
## Acknowledgements
- Thank you to Austin Morland for his Entity Component System design accompanied by fantastic documentation which can be found [here](https://austinmorlan.com/posts/entity_component_system/)
- Thank you to Lazy Foo' Productions for their SDL2 tutorial which can be found [here](https://lazyfoo.net/tutorials/SDL/)
- Thank you to Jeremiah for writing this [article](https://www.3dgep.com/understanding-quaternions/) on quaternions which I found to be the best explanation around