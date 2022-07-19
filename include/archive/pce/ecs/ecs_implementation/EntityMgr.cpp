#ifndef EntityMgr_cpp
#define EntityMgr_cpp

#include <bitset>
#include <queue>
#include <cassert>
#include <array>


// Entity alii and constants
using Entity = uint32_t;
const int MAX_ENTITIES = 5000;

// Component alii and constants
using ComponentType = std::uint8_t;
const int MAX_COMPS = 20;

// Signature alii
using Signature = std::bitset<MAX_COMPS>;


/* -------------------- Entity Manager Class -------------------*/

class EntityMgr
{
public:

    EntityMgr() 
    {
        // Fill Available Entities queue with all possible entities 
        for (int i = 0; i < MAX_ENTITIES; ++i) 
        {
            availableEntities.push(i);
        }
    }

    Entity CreateEntity()
    {   
        // ensure room is available
        assert(activeEntityCount < MAX_ENTITIES);

        Entity id = availableEntities.front();
        availableEntities.pop();
        ++activeEntityCount;

        return id;
    }

    void DestroyEntity(Entity entity)
    {
        // Ensure entity is in set of those which can be deleted
        assert(entity < MAX_ENTITIES);

        signatures[entity].reset();
        availableEntities.push(entity);
        --activeEntityCount;
    }
    
    void SetSignature(Entity entity, Signature signature)
    {
        assert(entity < MAX_ENTITIES);
        signatures[entity] = signature;
    }

    Signature GetSignature(Entity entity)
    {
        return signatures[entity];
    }

private:
    std::queue<int> availableEntities;
    std::array<Signature, MAX_ENTITIES> signatures{};

    int activeEntityCount{};
};






#endif /* EntityMgr_cpp */
