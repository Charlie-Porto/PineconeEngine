#ifndef base_entity_forging_hpp
#define base_entity_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
base entity forging. 
-----------------------------------------------------------------*/

extern ControlPanel control;

namespace pce3d {
namespace forge {
  
  uint32_t forgeBaseEntity(
      const glm::dvec3 center_location
  );

}
}




#endif /* base_entity_forging_hpp */
