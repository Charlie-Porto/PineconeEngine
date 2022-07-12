#ifndef surface_component_cpp
#define surface_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing basic information about an entity's surface
(assumes surface is consistent in charateristics)
-----------------------------------------------------------------*/

#include <vector>

namespace pce {

struct Surface {
  std::vector<int> natural_color;
  double luminosity;
  double reflectiveness;
  double transparency;
};

}

#endif /* surface_component_cpp */
