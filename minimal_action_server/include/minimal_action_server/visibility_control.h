#ifndef MINIMAL_ACTION_SERVER__VISIBILITY_CONTROL_H_
#define MINIMAL_ACTION_SERVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MINIMAL_ACTION_SERVER_EXPORT __attribute__ ((dllexport))
    #define MINIMAL_ACTION_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define MINIMAL_ACTION_SERVER_EXPORT __declspec(dllexport)
    #define MINIMAL_ACTION_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MINIMAL_ACTION_SERVER_BUILDING_LIBRARY
    #define MINIMAL_ACTION_SERVER_PUBLIC MINIMAL_ACTION_SERVER_EXPORT
  #else
    #define MINIMAL_ACTION_SERVER_PUBLIC MINIMAL_ACTION_SERVER_IMPORT
  #endif
  #define MINIMAL_ACTION_SERVER_PUBLIC_TYPE MINIMAL_ACTION_SERVER_PUBLIC
  #define MINIMAL_ACTION_SERVER_LOCAL
#else
  #define MINIMAL_ACTION_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define MINIMAL_ACTION_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define MINIMAL_ACTION_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define MINIMAL_ACTION_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MINIMAL_ACTION_SERVER_PUBLIC
    #define MINIMAL_ACTION_SERVER_LOCAL
  #endif
  #define MINIMAL_ACTION_SERVER_PUBLIC_TYPE
#endif

#endif  // MINIMAL_ACTION_SERVER__VISIBILITY_CONTROL_H_
