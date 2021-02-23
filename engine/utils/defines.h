#pragma once

#if defined(_MSC_VER)
    #define COMPILER_MSVC 1
    #define PRAGMA_DIAGNOSTIC_IGNORED(x) 
    #define PRAGMA_CLANG_DIAGNOSTIC_IGNORED(x) 
    #define PRAGMA_DIAGNOSTIC_PUSH() __pragma(warning (push))
    #define PRAGMA_DIAGNOSTIC_POP() __pragma(warning (pop))
    #define PARGMA_WARNING_DISABLE(x) __pragma(warning( disable : (x) ))
#elif defined(__clang__)
    #define COMPILER_CLANG 1
    #define DO_PRAGMA_IGNORED(x) _Pragma (#x)
    #define PRAGMA_DIAGNOSTIC_IGNORED(x) DO_PRAGMA_IGNORED(clang diagnostic ignored #x)
    // for clang specific diagnostics
    #define PRAGMA_CLANG_DIAGNOSTIC_IGNORED(x) PRAGMA_DIAGNOSTIC_IGNORED(x)
    #define PRAGMA_DIAGNOSTIC_PUSH() _Pragma ("clang diagnostic push")
    #define PRAGMA_DIAGNOSTIC_POP() _Pragma ("clang diagnostic pop")
    #define PARGMA_WARNING_DISABLE(x) 
#elif defined(__GNUC__)
    #define COMPILER_GCC 1
    #define DO_PRAGMA_IGNORED(x) _Pragma (#x)
    #define PRAGMA_DIAGNOSTIC_IGNORED(x) DO_PRAGMA_IGNORED(GCC diagnostic ignored #x)
    #define PRAGMA_CLANG_DIAGNOSTIC_IGNORED(x) 
    #define PRAGMA_DIAGNOSTIC_PUSH() _Pragma ("GCC diagnostic push")
    #define PRAGMA_DIAGNOSTIC_POP() _Pragma ("GCC diagnostic pop")
#else
    #error Unknown compiler used
#endif
