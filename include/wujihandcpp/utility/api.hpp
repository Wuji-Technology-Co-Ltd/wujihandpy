#pragma once

#ifdef _WIN32
# ifdef STATIC_LINKING_WUJIHANDCPP
#  define WUJIHANDCPP_API
# else
#  ifdef BUILDING_WUJIHANDCPP
#   define WUJIHANDCPP_API __declspec(dllexport)
#  else
#   define WUJIHANDCPP_API __declspec(dllimport)
#  endif
# endif
#else
# define WUJIHANDCPP_API __attribute__((visibility("default")))
#endif