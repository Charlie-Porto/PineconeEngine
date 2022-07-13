#ifndef sign_hpp
#define sign_hpp

namespace pce {
namespace math {

  template <typename T> T sign(T val) {
   return (T(0) < val) - (val < T(0));
  } 

}
}



#endif /* sign_hpp */
