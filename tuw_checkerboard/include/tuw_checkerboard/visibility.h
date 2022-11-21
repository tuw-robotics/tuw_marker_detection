#ifndef TUW_CHECKERBOARD__VISIBILITY_H_
#define TUW_CHECKERBOARD__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define TUW_CHECKERBOARD_EXPORT __attribute__ ((dllexport))
    #define TUW_CHECKERBOARD_IMPORT __attribute__ ((dllimport))
  #else
    #define TUW_CHECKERBOARD_EXPORT __declspec(dllexport)
    #define TUW_CHECKERBOARD_IMPORT __declspec(dllimport)
  #endif

  #ifdef TUW_CHECKERBOARD_DLL
    #define TUW_CHECKERBOARD_PUBLIC TUW_CHECKERBOARD_EXPORT
  #else
    #define TUW_CHECKERBOARD_PUBLIC TUW_CHECKERBOARD_IMPORT
  #endif

  #define TUW_CHECKERBOARD_PUBLIC_TYPE TUW_CHECKERBOARD_PUBLIC

  #define TUW_CHECKERBOARD_LOCAL

#else

  #define TUW_CHECKERBOARD_EXPORT __attribute__ ((visibility("default")))
  #define TUW_CHECKERBOARD_IMPORT

  #if __GNUC__ >= 4
    #define TUW_CHECKERBOARD_PUBLIC __attribute__ ((visibility("default")))
    #define TUW_CHECKERBOARD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TUW_CHECKERBOARD_PUBLIC
    #define TUW_CHECKERBOARD_LOCAL
  #endif

  #define TUW_CHECKERBOARD_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TUW_CHECKERBOARD__VISIBILITY_H_
