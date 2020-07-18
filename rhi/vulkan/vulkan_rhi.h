#pragma once

#include "rhi.h"
#include "gos_render.h"

namespace rhi_vulkan {
	bool initialize(graphics::RenderWindowHandle window);
	bool finalize();

	void make_current_context();
	unsigned int get_window_flags(void);
}
