#ifndef glm_hash_hpp
#define glm_hash_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
function that hashes a glm::dvec3 and thus allows it to be used as 
they key for a std::unordered_map
-----------------------------------------------------------------*/

#include <functional>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

template <>
struct std::hash<glm::dvec3>
{
    size_t operator()( const glm::dvec3& v ) const
    {
        std::size_t res = 17;
        res = res * 31 + std::hash<double>{}( v.x );
        res = res * 31 + std::hash<double>{}( v.y );
        res = res * 31 + std::hash<double>{}( v.z );
        return res;
    }
};


template <>
struct std::hash<glm::ivec3>
{
    size_t operator()( const glm::ivec3& v ) const
    {
        std::size_t res = 17;
        res = res * 31 + std::hash<int>{}( v.x );
        res = res * 31 + std::hash<int>{}( v.y );
        res = res * 31 + std::hash<int>{}( v.z );
        return res;
    }
};

template <>
struct std::hash<glm::dvec2> 
{
    size_t operator()( const glm::dvec2& v ) const
    {
        // std::size_t res = 17;
        // res = res * 31 + std::hash<double>{}( v.x );
        // res = res * 31 + std::hash<double>{}( v.y );
        std::size_t resa = 31 + std::hash<double>{}( v.x );
        std::size_t resb = 31 + std::hash<double>{}( v.y );

        return resa ^ (resb << 1);
    }
};





#endif /* glm_hash_hpp */
