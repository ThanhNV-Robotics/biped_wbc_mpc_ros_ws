#ifndef CONTROL_ACTIONS_CPP__VISIBILITY_CONTROL_H_
#define CONTROL_ACTIONS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROL_ACTIONS_CPP_EXPORT __attribute__ ((dllexport))
    #define CONTROL_ACTIONS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROL_ACTIONS_CPP_EXPORT __declspec(dllexport)
    #define CONTROL_ACTIONS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROL_ACTIONS_CPP_BUILDING_DLL
    #define CONTROL_ACTIONS_CPP_PUBLIC CONTROL_ACTIONS_CPP_EXPORT
  #else
    #define CONTROL_ACTIONS_CPP_PUBLIC CONTROL_ACTIONS_CPP_IMPORT
  #endif
  #define CONTROL_ACTIONS_CPP_PUBLIC_TYPE CONTROL_ACTIONS_CPP_PUBLIC
  #define CONTROL_ACTIONS_CPP_LOCAL
#else
  #define CONTROL_ACTIONS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CONTROL_ACTIONS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CONTROL_ACTIONS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROL_ACTIONS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROL_ACTIONS_CPP_PUBLIC
    #define CONTROL_ACTIONS_CPP_LOCAL
  #endif
  #define CONTROL_ACTIONS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_ACTIONS_CPP__VISIBILITY_CONTROL_H_