#pragma once

#include <cstdio>

#if defined(__has_include)
# if __has_include(<rclcpp/rclcpp.hpp>)

#  ifndef IS_ROS2_ENVIRONMENT
#   define IS_ROS2_ENVIRONMENT
#  endif

#  include <rclcpp/logger.hpp>
#  include <rclcpp/logging.hpp>
namespace wujihandcpp::utility {
inline rclcpp::Logger& get_logger_cached() {
    static rclcpp::Logger logger = rclcpp::get_logger("librmcs");
    return logger;
}
} // namespace wujihandcpp::utility

# endif
#endif

#ifndef LOG_INFO
# ifdef IS_ROS2_ENVIRONMENT
#  define LOG_INFO(...) RCLCPP_INFO(utility::get_logger_cached(), __VA_ARGS__)
# else
#  define LOG_INFO(format, ...) \
      std::fprintf(stdout, "[INFO] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
# endif
#endif

#ifndef LOG_WARN
# ifdef IS_ROS2_ENVIRONMENT
#  define LOG_WARN(...) RCLCPP_WARN(utility::get_logger_cached(), __VA_ARGS__)
# else
#  define LOG_WARN(format, ...) \
      std::fprintf(stderr, "[WARN] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
# endif
#endif

#ifndef LOG_ERROR
# ifdef IS_ROS2_ENVIRONMENT
#  define LOG_ERROR(...) RCLCPP_ERROR(utility::get_logger_cached(), __VA_ARGS__)
# else
#  define LOG_ERROR(format, ...) \
      std::fprintf(stderr, "[ERROR] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
# endif
#endif
