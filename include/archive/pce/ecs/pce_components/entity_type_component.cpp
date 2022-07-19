#ifndef entity_type_component_cpp
#define entity_type_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing an entity's type id
* this can be useful in the case that a system must treat different
  'entity types' differently 
-----------------------------------------------------------------*/


namespace pce {

struct EntityType {
  uint32_t type_id;
};

}

#endif /* entity_type_component_cpp */
