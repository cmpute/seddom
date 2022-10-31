#pragma once

// #define DISABLE_EASY_PROFILER
// #define ENABLE_THREAD_PROFILING

#if defined(PROFILING) && defined(BUILD_WITH_EASY_PROFILER)
    #include <easy/profiler.h>
    #include <easy/arbitrary_value.h>

    #define PROFILE_ENABLE_LOCAL EASY_PROFILER_ENABLE
    #define PROFILE_ENABLE_REMOTE \
        EASY_PROFILER_ENABLE;     \
        profiler::startListen()
    #define PROFILE_FUNCTION EASY_FUNCTION()
    #define PROFILE_BLOCK(s) EASY_BLOCK(s)
    #define PROFILE_SPLIT(s) \
        EASY_END_BLOCK;      \
        EASY_BLOCK(s)
    #define PROFILE_VALUE(n, v) EASY_VALUE(n, v)

    #ifdef ENABLE_THREAD_PROFILING
        #define PROFILE_THREAD_BLOCK(s) PROFILE_BLOCK(s)
        #define PROFILE_THREAD_SPLIT(s) PROFILE_SPLIT(s)
    #else
        #define PROFILE_THREAD_BLOCK(s)
        #define PROFILE_THREAD_SPLIT(s)
    #endif
#else
    #define PROFILE_ENABLE_LOCAL
    #define PROFILE_ENABLE_REMOTE
    #define PROFILE_FUNCTION
    #define PROFILE_BLOCK(s)
    #define PROFILE_SPLIT(s)
    #define PROFILE_VALUE(n, v)

    #define PROFILE_THREAD_BLOCK(s)
    #define PROFILE_THREAD_SPLIT(s)
#endif

#define ipow8(n) (1 << (3 * (n)))
constexpr double PI = 3.141592653589793238463;
constexpr double DPI = 6.283185307179586476925;
constexpr double SQ3 = 1.732050807568877293527;

#ifdef ROS_VERSION
    #define DEBUG_WRITE(s) ROS_DEBUG_STREAM(s)
    #define INFO_WRITE(s) ROS_INFO_STREAM(s)
    #define WARN_WRITE(s) ROS_INFO_STREAM(s)
#else
    #include <iostream>
    #ifndef NDEBUG
        #define DEBUG_WRITE(s) std::cout << "DEBUG: " << s << std::endl;
    #else
        #define DEBUG_WRITE(s)
    #endif
    #define INFO_WRITE(s) std::cout << "INFO: " << s << std::endl;
    #define WARN_WRITE(s) std::cout << "WARN: " << s << std::endl;
#endif
