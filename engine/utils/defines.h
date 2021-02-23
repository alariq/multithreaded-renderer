#pragma once

#if defined(_MSC_VER)
    #define COMPILER_MSVC 1
#elif defined(__clang__)

#elif defined(__GNUC__)

#else
    #error Unknown compiler used
#endif
