#pragma once

#include <cstdio>

#ifndef LOG_INFO
# define LOG_INFO(format, ...) \
     std::fprintf(stdout, "[INFO] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#endif

#ifndef LOG_WARN
# define LOG_WARN(format, ...) \
     std::fprintf(stderr, "[WARN] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#endif

#ifndef LOG_ERROR
# define LOG_ERROR(format, ...) \
     std::fprintf(stderr, "[ERROR] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#endif
