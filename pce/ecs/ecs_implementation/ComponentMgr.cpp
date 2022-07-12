#ifndef ComponentMgr_cpp
#define ComponentMgr_cpp

// STD lib
#include <typeinfo>
#include <unordered_map>
#include <memory>

// other modules
#include "ComponentArray.cpp"

class ComponentMgr 
{
public:
    template<typename T>
    void RegisterComponent()
    {
        const char* type_name = typeid(T).name();
        
        assert(component_types.find(type_name) == component_types.end());

        // add component to component type map
        component_types.insert({type_name, next_component_type});

        // create a pointer to the ComponentArray, and add to array map
        component_arrays.insert({type_name, std::make_shared<ComponentArray<T>>()});

        // incrememt component type ID
        ++next_component_type;
    }

    template<typename T>
    int GetComponentType()
    {
        const char* type_name = typeid(T).name();
        
        return component_types[type_name];
    }

    
    template<typename T>
    void AddComponent(Entity entity, T component)
    {
        GetComponentArray<T>()->InsertData(entity, component);
    }

    template<typename T>
    void RemoveComponent(Entity entity)
    {
        GetComponentArray<T>()->RemoveData(entity);
    }

    template<typename T>
    T& GetComponent(Entity entity)
    {
        return GetComponentArray<T>()->GetData(entity);
    }

    // Destroy 
    void EntityDestroyed(Entity entity)
    {
        for (auto const& pair : component_arrays)
        {
            auto const& component = pair.second;
            component->EntityDestroyed(entity);
        }
    }

private:
    // Map of string to component array 
    std::unordered_map<const char*, std::shared_ptr<IComponentArray>> component_arrays{};

    // Map of string to component type
    std::unordered_map<const char*, int> component_types{};

    int next_component_type{};

    // function to get pointer (to type T)
    template<typename T>
    std::shared_ptr<ComponentArray<T>> GetComponentArray()
    {
        const char* type_name = typeid(T).name();

        // this return converts the IComponentArray pointer to a ComponentArray pointer (damn. sexy)
        return std::static_pointer_cast<ComponentArray<T>>(component_arrays[type_name]);
    }
};





#endif /* ComponentMgr_cpp */
