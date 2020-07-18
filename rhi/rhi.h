#pragma once

#include "gos_render.h"


struct rhi {

	struct RenderContext {
		graphics::RenderWindowHandle rw_handle_;
		void* platform_data_;
	};
	typedef RenderContext*   RenderContextHandle;

	bool (*initialize_rhi)(graphics::RenderWindowHandle window);
	bool (*finalize_rhi)();
	unsigned int (*get_window_flags)(void);

	void (*make_current_context)(void);
};

