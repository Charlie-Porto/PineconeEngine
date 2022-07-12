#ifndef sign_cpp
#define sign_cpp

namespace pce {
namespace math {

  template <typename T> T sign(T val) {
   return (T(0) < val) - (val < T(0));
  } 

}
}



#endif /* sign_cpp */
