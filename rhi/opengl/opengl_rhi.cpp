#include "opengl_rhi.h"
#include "opengl_device.h"
#include "rhi.h"
#include "gos_render.h"
#include "utils/logging.h"
#include "GL/glew.h"
#include "gameos.hpp"
#include "SDL2/SDL.h"
#include "Remotery/lib/Remotery.h"
#include <cstdio>
#include <cassert>

static bool VERBOSE_VIDEO = true;
static bool VERBOSE_RENDER = true;
static bool VERBOSE_MODES = true;
static bool ENABLE_VSYNC = true;
static unsigned int RENDER_BACKEND = SDL_WINDOW_VULKAN; // SDL_WINDOW_OPENGL

namespace graphics {
	SDL_Window* get_platform_window(RenderWindowHandle rw_handle);
};

namespace rhi_opengl {

	static rhi::RenderContextHandle g_ctx;

	const char *getStringForType(GLenum type) {
		switch (type) {
		case GL_DEBUG_TYPE_ERROR:
			return "DEBUG_TYPE_ERROR";
		case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
			return "DEBUG_TYPE_DEPRECATED_BEHAVIOR";
		case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
			return "DEBUG_TYPE_UNDEFINED_BEHAVIOR";
		case GL_DEBUG_TYPE_PERFORMANCE:
			return "DEBUG_TYPE_PERFORMANCE";
		case GL_DEBUG_TYPE_PORTABILITY:
			return "DEBUG_TYPE_PORTABILITY";
		case GL_DEBUG_TYPE_MARKER:
			return "DEBUG_TYPE_MARKER";
		case GL_DEBUG_TYPE_PUSH_GROUP:
			return "DEBUG_TYPE_PUSH_GROUP";
		case GL_DEBUG_TYPE_POP_GROUP:
			return "DEBUG_TYPE_POP_GROUP";
		case GL_DEBUG_TYPE_OTHER:
			return "DEBUG_TYPE_OTHER";
		default:
			return "(undefined)";
		}
	}

	const char *getStringForSource(GLenum type) {
		switch (type) {
		case GL_DEBUG_SOURCE_API:
			return "DEBUG_SOURCE_API";
		case GL_DEBUG_SOURCE_SHADER_COMPILER:
			return "DEBUG_SOURCE_SHADER_COMPILER";
		case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
			return "DEBUG_SOURCE_WINDOW_SYSTEM";
		case GL_DEBUG_SOURCE_THIRD_PARTY:
			return "DEBUG_SOURCE_THIRD_PARTY";
		case GL_DEBUG_SOURCE_APPLICATION:
			return "DEBUG_SOURCE_APPLICATION";
		case GL_DEBUG_SOURCE_OTHER:
			return "DEBUG_SOURCE_OTHER";
		default:
			return "(undefined)";
		}
	}

	const char *getStringForSeverity(GLenum type) {
		switch (type) {
		case GL_DEBUG_SEVERITY_HIGH:
			return "DEBUG_SEVERITY_HIGH";
		case GL_DEBUG_SEVERITY_MEDIUM:
			return "DEBUG_SEVERITY_MEDIUM";
		case GL_DEBUG_SEVERITY_LOW:
			return "DEBUG_SEVERITY_LOW";
		case GL_DEBUG_SEVERITY_NOTIFICATION:
			return "DEBUG_SEVERITY_NOTIFICATION";
		default:
			return "(undefined)";
		}
	}

	void GLAPIENTRY OpenGLDebugLog(GLenum source, GLenum type, GLuint id, GLenum severity,
								   GLsizei /*length*/, const GLchar *message,
								   const GLvoid *userParam) {
		(void)userParam;
		if (severity != GL_DEBUG_SEVERITY_NOTIFICATION && severity != GL_DEBUG_SEVERITY_LOW) {
			printf("Type: %s; Source: %s; ID: %d; Severity : %s\n", getStringForType(type),
				   getStringForSource(source), id, getStringForSeverity(severity));
			printf("Message : %s\n", message);
		}
	}

	//==============================================================================
	rhi::RenderContextHandle init_render_context(graphics::RenderWindowHandle rw_handle)
	{
		SDL_Window* platform_window = graphics::get_platform_window(rw_handle);
		SDL_GLContext glcontext = SDL_GL_CreateContext(platform_window);
		if (!glcontext ) {
	        fprintf(stderr, "SDL_GL_CreateContext(): %s\n", SDL_GetError());
			return NULL;
		}

		if (SDL_GL_MakeCurrent(platform_window, glcontext) < 0) {
			SDL_GL_DeleteContext(glcontext);
			return NULL;
		}

		if (ENABLE_VSYNC) {
			SDL_GL_SetSwapInterval(1);
		} else {
			SDL_GL_SetSwapInterval(0);
		}

		if (VERBOSE_RENDER) {
			SDL_DisplayMode mode;
			SDL_GetCurrentDisplayMode(0, &mode);
			printf("Current Display Mode:\n");
			printf("Screen BPP: %d\n", SDL_BITSPERPIXEL(mode.format));
			printf("\n");
			printf("Vendor     : %s\n", glGetString(GL_VENDOR));
			printf("Renderer   : %s\n", glGetString(GL_RENDERER));
			printf("Version    : %s\n", glGetString(GL_VERSION));
			const GLubyte *exts = glGetString(GL_EXTENSIONS);
			printf("Extensions : %s\n", exts);
			printf("\n");

			int value;
			int status = 0;

			/*
			   status = SDL_GL_GetAttribute(SDL_GL_RED_SIZE, &value);
			   if (!status) {
			   printf("SDL_GL_RED_SIZE: requested %d, got %d\n", 5, value);
			   } else {
			   printf("Failed to get SDL_GL_RED_SIZE: %s\n", SDL_GetError());
			   }
			   status = SDL_GL_GetAttribute(SDL_GL_GREEN_SIZE, &value);
			   if (!status) {
			   printf("SDL_GL_GREEN_SIZE: requested %d, got %d\n", 5, value);
			   } else {
			   printf("Failed to get SDL_GL_GREEN_SIZE: %s\n", SDL_GetError());
			   }
			   status = SDL_GL_GetAttribute(SDL_GL_BLUE_SIZE, &value);
			   if (!status) {
			   printf("SDL_GL_BLUE_SIZE: requested %d, got %d\n", 5, value);
			   } else {
			   printf("Failed to get SDL_GL_BLUE_SIZE: %s\n", SDL_GetError());
			   }
			   */
			// status = SDL_GL_GetAttribute(SDL_GL_DEPTH_SIZE, &value);
			// if (!status) {
			//    printf("SDL_GL_DEPTH_SIZE: requested %d, got %d\n", 16, value);
			//} else {
			//    printf("Failed to get SDL_GL_DEPTH_SIZE: %s\n", SDL_GetError());
			//}

			/*
			status = SDL_GL_GetAttribute(SDL_GL_MULTISAMPLEBUFFERS, &value);
			if (!status) {
				printf("SDL_GL_MULTISAMPLEBUFFERS: %d\n", value);
			} else {
				printf("Failed to get SDL_GL_MULTISAMPLEBUFFERS: %s\n",
						SDL_GetError());
			}

			status = SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES, &value);
			if (!status) {
				printf("SDL_GL_MULTISAMPLESAMPLES: %d\n", value);
			} else {
				printf("Failed to get SDL_GL_MULTISAMPLESAMPLES: %s\n",
						SDL_GetError());
			}
			*/
			status = SDL_GL_GetAttribute(SDL_GL_ACCELERATED_VISUAL, &value);
			if (!status) {
				printf("SDL_GL_ACCELERATED_VISUAL: %d\n", value);
			} else {
				printf("Failed to get SDL_GL_ACCELERATED_VISUAL: %s\n", SDL_GetError());
			}

			status = SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &value);
			if (!status) {
				printf("SDL_GL_CONTEXT_MAJOR_VERSION: %d\n", value);
			} else {
				printf("Failed to get SDL_GL_CONTEXT_MAJOR_VERSION: %s\n", SDL_GetError());
			}

			status = SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &value);
			if (!status) {
				printf("SDL_GL_CONTEXT_MINOR_VERSION: %d\n", value);
			} else {
				printf("Failed to get SDL_GL_CONTEXT_MINOR_VERSION: %s\n", SDL_GetError());
			}
		}

		rhi::RenderContext *rc = new rhi::RenderContext();
		rc->platform_data_ = glcontext;
		rc->rw_handle_ = rw_handle;

		return rc;
	}

	//==============================================================================
	void destroy_render_context(rhi::RenderContextHandle rc_handle)
	{
	    rhi::RenderContext* rc = (rhi::RenderContext*)rc_handle;
	    assert(rc);

		SDL_GLContext glcontext = (SDL_GLContext)rc->platform_data_;
	
	    SDL_GL_DeleteContext(glcontext);
	    delete rc;
	}	

	//==============================================================================
	void make_current_context()
	{
	    rhi::RenderContext* rc = (rhi::RenderContext*)g_ctx;
	    assert(rc);
		SDL_GLContext glcontext = (SDL_GLContext)rc->platform_data_;
	    SDL_GL_MakeCurrent(graphics::get_platform_window(rc->rw_handle_), glcontext);
	}

	uint32_t get_window_flags(void) {
		return SDL_WINDOW_OPENGL;
	}

	bool initialize(graphics::RenderWindowHandle rw_handle) {

		g_ctx = init_render_context(rw_handle);
		if (!g_ctx) return 1;

		make_current_context();

		GLenum err = glewInit();
		if (GLEW_OK != err) {
			SPEW(("GLEW", "Error: %s\n", glewGetErrorString(err)));
			return 1;
		}

		glEnable(GL_DEBUG_OUTPUT);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
		// glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 76, 1, "My debug group");
		glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
		glDebugMessageCallbackARB((GLDEBUGPROCARB)&OpenGLDebugLog, NULL);

		SPEW(("GRAPHICS", "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION)));
		// if ((!GLEW_ARB_vertex_program || !GLEW_ARB_fragment_program))
		//{
		//   SPEW(("GRAPHICS", "No shader program support\n"));
		//  return 1;
		//}

		if (!glewIsSupported("GL_VERSION_3_0")) {
			SPEW(("GRAPHICS", "Minimum required OpenGL version is 3.0\n"));
			return 1;
		}

		const char *glsl_version = (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION);
		SPEW(("GRAPHICS", "GLSL version supported: %s\n", glsl_version));

		int glsl_maj = 0, glsl_min = 0;
		sscanf(glsl_version, "%d.%d", &glsl_maj, &glsl_min);

		if (glsl_maj < 3 || (glsl_maj == 3 && glsl_min < 30)) {
			SPEW(("GRAPHICS", "Minimum required OpenGL version is 330 ES, current: %d.%d\n",
				  glsl_maj, glsl_min));
			return 1;
		}

		char version[16] = {0};
		snprintf(version, sizeof(version), "%d%d", glsl_maj, glsl_min);
		SPEW(("GRAPHICS", "Using %s shader version\n", version));

		rmt_BindOpenGL();

		return true;
	}

	bool finalize() { 
        rmt_UnbindOpenGL();
		return true;
	}

	IRHIDevice* create_device() {
		return new RHIDeviceGL();
	}

	void destroy_device(IRHIDevice* device) {
		assert(device);
		delete device;
	}

} // namespace rhi_opengl



bool CreateRHI_OpenGL(rhi* backend) {
    assert(backend);
    backend->initialize_rhi = rhi_opengl::initialize;
    backend->finalize_rhi = rhi_opengl::finalize;
	backend->make_current_context = rhi_opengl::make_current_context;
    backend->get_window_flags = rhi_opengl::get_window_flags;
    backend->create_device = rhi_opengl::create_device;
    backend->destroy_device = rhi_opengl::destroy_device;
    return true;
}
