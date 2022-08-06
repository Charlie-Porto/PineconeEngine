#ifndef orderTag_hpp
#define orderTag_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
small tag struct to add some abstraction to the render-ordering process
-----------------------------------------------------------------*/

namespace pce3d {

struct orderTag{
  uint32_t entity;
  double closest_vertex_distance;
  double farthest_vertex_distance;
};

}




#endif /* orderTag_hpp */
