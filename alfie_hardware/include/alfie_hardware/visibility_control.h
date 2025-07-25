
#ifndef ALFIE_HARDWARE__VISIBILITY_CONTROL_H_
#define ALFIE_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ALFIE_HARDWARE_EXPORT __attribute__((dllexport))
#define ALFIE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define ALFIE_HARDWARE_EXPORT __declspec(dllexport)
#define ALFIE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef ALFIE_HARDWARE_BUILDING_DLL
#define ALFIE_HARDWARE_PUBLIC ALFIE_HARDWARE_EXPORT
#else
#define ALFIE_HARDWARE_PUBLIC ALFIE_HARDWARE_IMPORT
#endif
#define ALFIE_HARDWARE_PUBLIC_TYPE ALFIE_HARDWARE_PUBLIC
#define ALFIE_HARDWARE_LOCAL
#else
#define ALFIE_HARDWARE_EXPORT __attribute__((visibility("default")))
#define ALFIE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define ALFIE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define ALFIE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define ALFIE_HARDWARE_PUBLIC
#define ALFIE_HARDWARE_LOCAL
#endif
#define ALFIE_HARDWARE_PUBLIC_TYPE
#endif

#endif  // BRICKPI3_MOTORS__VISIBILITY_CONTROL_H_
