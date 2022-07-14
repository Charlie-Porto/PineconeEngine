#ifndef SystemMgr_cpp
#define SystemMgr_cpp

// STL
#include <memory>
#include <unordered_map>

// Other
#include "System.cpp"
#include "ComponentMgr.cpp"



class SystemMgr
{
public:
    template <typename T>
    std::shared_ptr<T> RegisterSystem()
    {
        const char* type_name = typeid(T).name();

        // creates an instace of T and returns a shared pointer to it
        auto system = std::make_shared<T>();
        
        systems.insert({type_name, system});
        return system;
    }

    template<typename T>
    void SetSignature(Signature signature)
    {
        const char* type_name = typeid(T).name();
        signatures.insert({type_name, signature});
    }


    void EntityDestroyed(Entity entity)
    {
        for (auto const& pair : systems)
        {
            auto const& system = pair.second;
            system->entities.erase(entity);
        }
    }


    void EntitySignatureChanged(Entity entity, Signature entitySig)
    {
        for (auto const& pair : systems)
        {
            auto const& type = pair.first;
            auto const& system = pair.second;
            auto const& systemSig = signatures[type];
            bool if_retain_entity = true;

            // check if system components are present in entity sig
            for (size_t i = 0; i < systemSig.size(); ++i) {
                if (systemSig[i]) {
                    if (!entitySig[i]) {
                        if_retain_entity = false;
                        break;
                    }
                }
            }
            if (!if_retain_entity) {
                system->entities.erase(entity);
            } else {
                system->entities.insert(entity);
            }

        }
    }

private:
    std::unordered_map<const char*, Signature> signatures{};
    std::unordered_map<const char*, std::shared_ptr<ISystem>> systems{};
};



#endif /* SystemMgr_cpp */
