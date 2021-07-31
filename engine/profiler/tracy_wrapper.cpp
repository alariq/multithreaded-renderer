#include "utils/defines.h"

PRAGMA_DIAGNOSTIC_PUSH()
PRAGMA_DIAGNOSTIC_IGNORED(-Wshadow)
// this is bad to disable it, but somehow clang complains, although looks like it is false
// positive
PRAGMA_DIAGNOSTIC_IGNORED(-Warray-bounds)

#include "Tracy/TracyClient.cpp" 

PRAGMA_DIAGNOSTIC_POP()
