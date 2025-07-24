
#ifndef KITT_HARDWARE__VISIBILITY_CONTROL_H_
#define KITT_HARDWRE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define KITT_HARDWARE_EXPORT __attribute__((dllexport))
#define KITT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define KITT_HARDWARE_EXPORT __declspec(dllexport)
#define KITT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef KITT_HARDWARE_BUILDING_DLL
#define KITT_HARDWARE_PUBLIC KITT_HARDWARE_EXPORT
#else
#define KITT_HARDWARE_PUBLIC KITT_HARDWARE_IMPORT
#endif
#define KITT_HARDWARE_PUBLIC_TYPE KITT_HARDWARE_PUBLIC
#define KITT_HARDWARE_LOCAL
#else
#define KITT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define KITT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define KITT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define KITT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define KITT_HARDWARE_PUBLIC
#define KITT_HARDWARE_LOCAL
#endif
#define KITT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // BRICKPI3_MOTORS__VISIBILITY_CONTROL_H_
