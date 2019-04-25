#include "gameos.hpp"
#include "gos_render.h"
#include <stdio.h>
#include <time.h>
#include <queue>

#include <SDL2/SDL.h>
#include "gos_input.h"

#include "utils/camera.h"
#include "utils/shader_builder.h"
#include "utils/gl_utils.h"
#include "utils/timing.h"
#include "utils/threading.h"

#include "Remotery/lib/Remotery.h"

#include <signal.h>

extern void gos_CreateRenderer(graphics::RenderContextHandle ctx_h, graphics::RenderWindowHandle win_h, int w, int h);
extern void gos_DestroyRenderer();
extern void gos_RendererBeginFrame();
extern void gos_RendererEndFrame();
extern void gos_RendererHandleEvents();
extern void gos_RenderUpdateDebugInput();

extern bool gosExitGameOS();

static bool g_exit = false;
static bool g_focus_lost = false;
bool g_debug_draw_calls = false;
static camera g_camera;


// TODO: implement GPU/CPU bound stats (based on who wait for whom)

const uint32_t NUM_BUFFERED_FRAMES = 1;
threading::Event* g_main_event[NUM_BUFFERED_FRAMES];
threading::Event* g_render_event[NUM_BUFFERED_FRAMES];

SDL_Thread *g_render_thread = 0;
bool g_rendering = true;
int RenderThreadMain(void* data);

class R_job {
public:
    virtual void exec() = 0;
    virtual ~R_job() {}
};

class R_wait_event: public R_job {
    threading::Event* ev_;
    int ev_idx_;

    R_wait_event(const R_wait_event& rwe);
    void operator=(const R_wait_event& rwe);

public:
    R_wait_event(threading::Event* ev, int ev_idx):ev_(ev), ev_idx_(ev_idx) {}

    void exec() {
        printf("RT: WAIT FOR EVENT [%d]\n", ev_idx_);
        rmt_ScopedCPUSample(RT_wait, 0);
        ev_->Wait();
        printf("RT: WAIT FOR EVENT DONE [%d]\n", ev_idx_);
    }
};

class R_signal_event: public R_job {
    threading::Event* ev_;
    int ev_idx_;

    // TODO: use = delete
    R_signal_event(const R_signal_event& rwe);
    void operator=(const R_signal_event& rwe);
public:
    R_signal_event(threading::Event* ev, int ev_idx):ev_(ev), ev_idx_(ev_idx) {}

    void exec() {
        rmt_ScopedCPUSample(RT_signal, 0);
        printf("RT: SIGNAL EVENT [%d]\n", ev_idx_);
        ev_->Signal();
        printf("RT: SIGNAL EVENT DONE [%d]\n", ev_idx_);
    }
};

class R_draw_job: public R_job {
    std::string name_;
    uint64_t sleep_millisec_;
public:    
    R_draw_job(const std::string& name, uint64_t sleep_millisec):
        name_(name), sleep_millisec_(sleep_millisec) {}

    void exec() {
        rmt_ScopedCPUSample(draw, 0);
        timing::sleep(sleep_millisec_ * 1000000);
    }
};

class R_scope_begin: public R_job {
    std::string name_;
public:    
    R_scope_begin(const std::string& name):
        name_(name){}

    void exec() {
        rmt_BeginCPUSampleDynamic(name_.c_str(), 0);
    }
};

class R_scope_end: public R_job {
public:    
    R_scope_end() {}

    void exec() {
        rmt_EndCPUSample();
    }
};

class RenderJobQueue {
    SDL_mutex* mutex_;
    std::queue<R_job*> job_list_;

public:
    RenderJobQueue() {
        mutex_ = SDL_CreateMutex();
    }

    ~RenderJobQueue() {
        SDL_DestroyMutex(mutex_);
    }

    void push(R_job* job) {
        SDL_LockMutex(mutex_);
        job_list_.push(job);
        SDL_UnlockMutex(mutex_);
    }

    R_job* pop() {
        R_job* pjob = 0;
        SDL_LockMutex(mutex_);
        if(!job_list_.empty()) {
            pjob = job_list_.front();
            job_list_.pop();
        }
        SDL_UnlockMutex(mutex_);
        return pjob;
    }
};

RenderJobQueue* g_render_job_queue = 0;

input::MouseInfo g_mouse_info;
input::KeyboardInfo g_keyboard_info;

static void handle_key_down( SDL_Keysym* keysym ) {
    switch( keysym->sym ) {
        case SDLK_ESCAPE:
            //if(keysym->mod & KMOD_ALT)
                g_exit = true;
            break;
        case 'd':
            if(keysym->mod & KMOD_RALT)
                g_debug_draw_calls = true;
            break;
    }
}

static void process_events( void ) {

    beginUpdateMouseState(&g_mouse_info);

    SDL_Event event;
    while( SDL_PollEvent( &event ) ) {

        if(g_focus_lost) {
            if(event.type != SDL_WINDOWEVENT_FOCUS_GAINED) {
                continue;
            } else {
                g_focus_lost = false;
            }
        }

        switch( event.type ) {
        case SDL_KEYDOWN:
            handle_key_down( &event.key.keysym );
            // fallthrough
        case SDL_KEYUP:
            handleKeyEvent(&event, &g_keyboard_info);
            break;
        case SDL_QUIT:
            g_exit = true;
            break;
		case SDL_WINDOWEVENT_RESIZED:
			{
				float w = (float)event.window.data1;
				float h = (float)event.window.data2;
				glViewport(0, 0, (GLsizei)w, (GLsizei)h);
                SPEW(("INPUT", "resize event: w: %f h:%f\n", w, h));
			}
			break;
        case SDL_WINDOWEVENT_FOCUS_LOST:
            g_focus_lost = true;
            break;
        case SDL_MOUSEMOTION:
            input::handleMouseMotion(&event, &g_mouse_info); 
            break;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
            //input::handleMouseButton(&event, &g_mouse_info);
            break;
        case SDL_MOUSEWHEEL:
            input::handleMouseWheel(&event, &g_mouse_info);
            break;
        }
    }

    input::updateMouseState(&g_mouse_info);
    input::updateKeyboardState(&g_keyboard_info);
}

extern bool g_disable_quads;

static void draw_screen( void )
{
    g_disable_quads = false;
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glCullFace(GL_FRONT);
    //CHECK_GL_ERROR;
    
	const int viewport_w = Environment.drawableWidth;
	const int viewport_h = Environment.drawableHeight;
    glViewport(0, 0, viewport_w, viewport_h);

    mat4 proj;
    g_camera.get_projection(&proj);
    mat4 viewM;
    g_camera.get_view(&viewM);

#if 1


    gos_VERTEX q[4];
    q[0].x = 10; q[0].y = 10;
    q[0].z = 0.0;
    q[0].rhw = 1;
    q[0].argb = 0xffff0000;
    q[1] = q[2] = q[3] = q[0];

    q[1].x = 210; q[1].y = 10;
    q[2].x = 110; q[2].y = 210;
    q[3].x = 10; q[3].y = 210;

    g_disable_quads = false;
    gos_DrawQuads(&q[0], 4);
    g_disable_quads = true;
#endif

    CHECK_GL_ERROR;

    // TODO: reset all states to sane defaults!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    glDepthMask(GL_TRUE);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    gos_RendererBeginFrame();
    Environment.UpdateRenderers();
    gos_RendererEndFrame();

    glUseProgram(0);
    //CHECK_GL_ERROR;
}

extern float frameRate;


const char* getStringForType(GLenum type)
{
	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR: return "DEBUG_TYPE_ERROR";
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: return "DEBUG_TYPE_DEPRECATED_BEHAVIOR";
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: return "DEBUG_TYPE_UNDEFINED_BEHAVIOR";
	case GL_DEBUG_TYPE_PERFORMANCE: return "DEBUG_TYPE_PERFORMANCE";
	case GL_DEBUG_TYPE_PORTABILITY: return "DEBUG_TYPE_PORTABILITY";
	case GL_DEBUG_TYPE_MARKER: return "DEBUG_TYPE_MARKER";
	case GL_DEBUG_TYPE_PUSH_GROUP: return "DEBUG_TYPE_PUSH_GROUP";
	case GL_DEBUG_TYPE_POP_GROUP: return "DEBUG_TYPE_POP_GROUP";
	case GL_DEBUG_TYPE_OTHER: return "DEBUG_TYPE_OTHER";
	default: return "(undefined)";
	}
}

const char* getStringForSource(GLenum type)
{
	switch (type)
	{
	case GL_DEBUG_SOURCE_API: return "DEBUG_SOURCE_API";
	case GL_DEBUG_SOURCE_SHADER_COMPILER: return "DEBUG_SOURCE_SHADER_COMPILER";
	case GL_DEBUG_SOURCE_WINDOW_SYSTEM: return "DEBUG_SOURCE_WINDOW_SYSTEM";
	case GL_DEBUG_SOURCE_THIRD_PARTY: return "DEBUG_SOURCE_THIRD_PARTY";
	case GL_DEBUG_SOURCE_APPLICATION: return "DEBUG_SOURCE_APPLICATION";
	case GL_DEBUG_SOURCE_OTHER: return "DEBUG_SOURCE_OTHER";
	default: return "(undefined)";
	}
}

const char* getStringForSeverity(GLenum type)
{
	switch (type)
	{
	case GL_DEBUG_SEVERITY_HIGH: return "DEBUG_SEVERITY_HIGH";
	case GL_DEBUG_SEVERITY_MEDIUM: return "DEBUG_SEVERITY_MEDIUM";
	case GL_DEBUG_SEVERITY_LOW: return "DEBUG_SEVERITY_LOW";
	case GL_DEBUG_SEVERITY_NOTIFICATION: return "DEBUG_SEVERITY_NOTIFICATION";
	default: return "(undefined)";
	}
}
//typedef void (GLAPIENTRY *GLDEBUGPROCARB)(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);
#ifdef PLATFORM_WINDOWS
void GLAPIENTRY OpenGLDebugLog(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
#else
void GLAPIENTRY OpenGLDebugLog(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, GLvoid* userParam)
#endif
{
	if (severity != GL_DEBUG_SEVERITY_NOTIFICATION && severity != GL_DEBUG_SEVERITY_LOW)
	{
		printf("Type: %s; Source: %s; ID: %d; Severity : %s\n",
			getStringForType(type),
			getStringForSource(source),
			id,
			getStringForSeverity(severity)
		);
		printf("Message : %s\n", message);
	}
}

#ifndef DISABLE_GAMEOS_MAIN
int main(int argc, char** argv)
{
    //signal(SIGTRAP, SIG_IGN);

    // gather command line
    uint32_t cmdline_len = 0;
    for(int i=0;i<argc;++i) {
        cmdline_len += strlen(argv[i]);
        cmdline_len += 1; // ' '
    }
    char* cmdline = new char[cmdline_len + 1];
    uint32_t offset = 0;
    for(int i=0;i<argc;++i) {
        uint32_t arglen = strlen(argv[i]);
        memcpy(cmdline + offset, argv[i], arglen);
        cmdline[offset + arglen] = ' ';
        offset += arglen + 1;
    }
    cmdline[cmdline_len] = '\0';

    // fills in Environment structure
    GetGameOSEnvironment(cmdline);

    delete[] cmdline;
    cmdline = NULL;

    int w = Environment.screenWidth;
    int h = Environment.screenHeight;

    Remotery* rmt;
    rmt_CreateGlobalInstance(&rmt);

    g_render_job_queue = new RenderJobQueue();

    SPEW(("INIT", "Starting render thread...\n"));
    int         threadReturnValue;
 
    g_render_thread = SDL_CreateThread(RenderThreadMain, "RenderThread", (void *)NULL);
    if (NULL == g_render_thread) {
        SPEW(("Render", "SDL_CreateThread failed: %s\n", SDL_GetError()));
    } else {
        SPEW(("Render", "[OK] STATUS: %d\n", threadReturnValue));
    }

    graphics::RenderWindowHandle win = graphics::create_window("mc2", w, h);
    if(!win)
        return 1;

    graphics::RenderContextHandle ctx = graphics::init_render_context(win);
    if(!ctx)
        return 1;

    graphics::make_current_context(ctx);

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        SPEW(("GLEW", "Error: %s\n", glewGetErrorString(err)));
        return 1;
    }

	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	//glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 76, 1, "My debug group");
	glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageCallbackARB(&OpenGLDebugLog, NULL);


    SPEW(("GRAPHICS", "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION)));
    //if ((!GLEW_ARB_vertex_program || !GLEW_ARB_fragment_program))
    //{
     //   SPEW(("GRAPHICS", "No shader program support\n"));
      //  return 1;
    //}

    if(!glewIsSupported("GL_VERSION_3_0")) {
        SPEW(("GRAPHICS", "Minimum required OpenGL version is 3.0\n"));
        return 1;
    }

    const char* glsl_version = (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
    SPEW(("GRAPHICS", "GLSL version supported: %s\n", glsl_version));

    int glsl_maj = 0, glsl_min = 0;
    sscanf(glsl_version, "%d.%d", &glsl_maj, &glsl_min);

    if(glsl_maj < 3 || (glsl_maj==3 && glsl_min < 30) ) {
        SPEW(("GRAPHICS", "Minimum required OpenGL version is 330 ES, current: %d.%d\n", glsl_maj, glsl_min));
        return 1;
    }

    char version[16] = {0};
    snprintf(version, sizeof(version), "%d%d", glsl_maj, glsl_min);
    SPEW(("GRAPHICS", "Using %s shader version\n", version));

    gos_CreateRenderer(ctx, win, w, h);

    Environment.InitializeGameEngine();

	float aspect = (float)w/(float)h;
	mat4 proj_mat = frustumProjMatrix(-aspect*0.5f, aspect*0.5f, -.5f, .5f, 1.0f, 100.0f);
	g_camera.set_projection(proj_mat);
	g_camera.set_view(mat4::translation(vec3(0, 0, -16)));

	timing::init();

    for(int i=0; i<NUM_BUFFERED_FRAMES;++i) {
        g_main_event[i] = new threading::Event();
        g_render_event[i] = new threading::Event();
        g_render_event[i]->Signal();
    }

    int ev_index = NUM_BUFFERED_FRAMES - 1;
    uint32_t render_frame_number = 0;
    while( !g_exit ) {

        char frnum_str[32] = {0};
        sprintf(frnum_str, "Frame: %d", render_frame_number);
        rmt_BeginCPUSampleDynamic(frnum_str, 0);

        SPEW(("SYNC", "Main: sync[%d]->Wait\n", ev_index));
        {
            rmt_ScopedCPUSample(WaitRender, 0);
            g_render_event[ev_index]->Wait();
        }

		uint64_t start_tick = timing::gettickcount();
		//timing::sleep(10*1000000);

        {
            rmt_ScopedCPUSample(DoGameLogic, 0);
            Environment.DoGameLogic();
		    //timing::sleep(10*1000000);
        }

        process_events();

        {
            rmt_ScopedCPUSample(RenderFrameCreation, 0);
		    //timing::sleep(5*1000000);

            // start render frame
            g_render_job_queue->push( new R_scope_begin(frnum_str) );
            g_render_job_queue->push( new R_wait_event(g_main_event[ev_index], ev_index) );

            gos_RendererHandleEvents();

            const uint32_t num_draw_calls = (rand()%3) + 1;
            for(int i=0; i<num_draw_calls;++i) {
                g_render_job_queue->push( new R_draw_job(std::string("DIP"), (rand()%2) + 1) );
            }

            graphics::make_current_context(ctx);
            draw_screen();
            graphics::swap_window(win);

            g_render_job_queue->push( new R_signal_event(g_render_event[ev_index], ev_index) );
            g_render_job_queue->push( new R_scope_end() );
        }

        SPEW(("SYNC", "Main sync[%d]->Signal\n", ev_index));
        {
            rmt_ScopedCPUSample(MainSignal, 0);
            g_main_event[ev_index]->Signal();
        }
        ev_index = (ev_index+1) % NUM_BUFFERED_FRAMES;

		//timing::sleep(8*1000000);

        render_frame_number++;

        g_exit |= gosExitGameOS();

		uint64_t end_tick = timing::gettickcount();
		uint64_t dt = timing::ticks2ms(end_tick - start_tick);
		frameRate = 1000.0f / (float)dt;

        rmt_EndCPUSample();
    }
    
    Environment.TerminateGameEngine();


    g_rendering = false;
    SPEW(("EXIT", "Waiting for render thread to finish"));
    SDL_WaitThread(g_render_thread, &threadReturnValue);

    delete g_render_job_queue;

    gos_DestroyRenderer();

    graphics::destroy_render_context(ctx);
    graphics::destroy_window(win);

    return 0;
}
#endif // DISABLE_GAMEOS_MAIN



int RenderThreadMain(void* data) {

    while(g_rendering) {
        R_job* pjob = g_render_job_queue->pop();
        if(pjob)
        {
            pjob->exec();
        }
    };

    printf("RenderThread Exit\n");

}
