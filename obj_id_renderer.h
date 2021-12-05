#pragma once

#include "engine/gameos.hpp"
#include "engine/utils/gl_utils.h"
#include <cstdint>

class ObjIdRenderer {

	const static int num_buffers_ = 3;

	typedef struct BufferData {
		GLuint obj_id_fbo_ = 0;
		DWORD gos_obj_id_rt = 0;
		GLuint obj_id_rt = 0;
		bool has_data_ = false;
	} BufferData;

	BufferData buf_;

	GLuint pbos_[num_buffers_];
    int cur_pbo_ = 0;

	GLuint width_ = 0;
	GLuint height_ = 0;

  public:
	bool Init(uint32_t width, uint32_t height);
	void Deinit();
	void Render(struct RenderFrameContext* rfc, GLuint scene_depth);
	uint32_t Readback(uint32_t x, uint32_t y);
};
