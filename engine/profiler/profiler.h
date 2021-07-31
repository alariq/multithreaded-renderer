/* interface for different profilers */

#pragma once

// setup through cmake
//#define USE_TRACY
//#define USE_REMOTERY

#if defined(USE_TRACY)
#define ENABLE_STATISTICS 1

#include "engine/Tracy/Tracy.hpp"
#include "engine/Tracy/TracyC.h"

#include "engine/utils/graphics.h"
#include "engine/Tracy/TracyOpenGL.hpp"

#define prof_initialize()
#define prof_finalize()

typedef TracyCZoneCtx prof_zone_id;

#define SCOPED_ZONE(flags) ZoneScoped()

// named zone
#define SCOPED_ZONE_N(name, flags) ZoneScopedN(#name)
// also creates named variable to avoid compiler warning related to variable shadowing if
// several SCOPED_ZONE_N are used
#define SCOPED_ZONE_NAMED(name, flags) ZoneNamedN(name, #name, true)

// can be useful, but Remotery does not support this
#if 0
#define SCOPED_ZONE_DYNAMIC_N(str_name, flags) \
    ZoneScoped(true); \
    ZoneName((str_name), strlen(str_name))
#endif

#define BEGIN_ZONE_N(ctx, name, flags) \
    TracyCZoneN(ctx, #name, true); \

#define BEGIN_ZONE_DYNAMIC_N(ctx, str_name, flags) \
    TracyCZoneN(ctx, #ctx, true); \
    TracyCZoneText(ctx, (str_name), strlen(str_name))

#define END_ZONE(ctx) TracyCZoneEnd(ctx)

#define FRAME_MARK() FrameMark
#define PROF_SET_THREAD_NAME(name) TracyCSetThreadName(name)

#define SCOPED_GPU_ZONE(name) TracyGpuZone(#name)

#define PROF_INIT_OPENGL() TracyGpuContext
#define PROF_FINALIZE_OPENGL()
#define PROF_GPU_TICK() TracyGpuCollect


#elif defined(USE_REMOTERY)


#include "Remotery/lib/Remotery.h"

typedef int prof_zone_id;

#define prof_initialize() \
{\
    Remotery* rmt;\
    rmt_CreateGlobalInstance(&rmt);\
}

#define prof_finalize()

#define SCOPED_ZONE() rmt_ScopedCPUSample(__FUNCTION__, 0)

#define SCOPED_ZONE_N(name, flags) rmt_ScopedCPUSample(name, flags)
#define SCOPED_ZONE_NAMED(name, flags) rmt_ScopedCPUSample(name, flags)

#define BEGIN_ZONE_N(ctx, name, flags) \
    int name = -1; (void)name; /* unused*/\
    rmt_BeginCPUSample(name, flags)

#define BEGIN_ZONE_DYNAMIC_N(name, str_name, flags) \
    int name = -1; (void)name; /* unused*/\
    rmt_BeginCPUSampleDynamic(str_name, flags)

#define END_ZONE(name) rmt_EndCPUSample()

#define FRAME_MARK()
#define PROF_SET_THREAD_NAME(name)

#define SCOPED_GPU_ZONE(name) \
    rmt_ScopedOpenGLSample(name)

#define PROF_INIT_OPENGL() rmt_BindOpenGL()
#define PROF_FINALIZE_OPENGL() rmt_UnbindOpenGL()
#define PROF_GPU_TICK() 

#else

typedef int prof_zone_id;

#define prof_initialize()

#define prof_finalize()

#define SCOPED_ZONE()
#define SCOPED_ZONE_N(name, flags)

#define BEGIN_ZONE_N(name, str_name, flags)\
    int name = -1; (void)name; /* unused*/

#define BEGIN_ZONE_DYNAMIC_N(name, str_name, flags)\
    int name = -1; (void)name; /* unused*/

#define END_ZONE(name)

#define FRAME_MARK()
#define PROF_SET_THREAD_NAME(name)

#define SCOPED_GPU_ZONE(name)

#define PROF_INIT_OPENGL() 
#define PROF_FINALIZE_OPENGL()
#define PROF_GPU_TICK() 

#endif


