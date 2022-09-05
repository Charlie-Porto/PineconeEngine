#ifndef CollisionReport_hpp
#define CollisionReport_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
struct for passing results of a collision from functions
-----------------------------------------------------------------*/

#include "CollisionTypeEnumeration.hpp"

namespace pce3d {
namespace collision {

struct CollisionReport{
  bool collision_occuring;
  glm::dvec3 point_of_contact;
  CollisionType a_collision_type; 
  uint32_t entity_a;
  uint32_t a_collision_type_area_id;
  uint32_t entity_b;
  CollisionType b_collision_type; 
  uint32_t b_collision_type_area_id;
};

}
}








#endif /* CollisionReport_hpp */
