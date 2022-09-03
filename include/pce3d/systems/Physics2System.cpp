#ifndef Physics2System_cpp
#define Physics2System_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
updated physics system
-----------------------------------------------------------------*/

#include <algorithm>
#include <iostream>
#include <pcecs/ecs/System.cpp>

/* include Core Manager for access to time */
#include <pceSDL/core/CoreManager.hpp>


extern ControlPanel control;

namespace pce3d {
class Physics2System : public ISystem {
public:

  void UpdateEntities() 
  {
    for (auto const& entity : entities) 
    {

    }
  }



private:
};
}
#endif /* Physics2System_cpp */
