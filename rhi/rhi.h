#pragma once

#include "gos_render.h"
#include "utils/vec.h"
#include <stdint.h>

enum class RHIQueueType: uint32_t {
	kUnknown = 0x0,
	kGraphics = 0x1,
	kCompute = 0x2,
	kPresentation = 0x4,
};

enum class RHIImageAspect : uint32_t {
	kColor = 0x1,
	kDepth = 0x2,
	kStencil = 0x4
};

class IRHIImage {
public:
};
//
//class IRHIQueue {
//public:
//	virtual void Clear(IRHIRenderTarget* rt, const vec4& color) = 0;
//	virtual ~IRHIQueue() = 0 {};
//};

class IRHICmdBuf {
public:

	virtual bool Begin() = 0;
	virtual bool End() = 0;
	virtual void Barrier_ClearToPresent(IRHIImage* image) = 0;
	virtual void Barrier_PresentToClear(IRHIImage* image) = 0;
	virtual void Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) = 0;
	virtual ~IRHICmdBuf() = 0 {};
};

class IRHIDevice {
public:

	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) = 0;

	virtual IRHIImage* GetFrameBuffer() = 0;
	virtual bool Submit(IRHICmdBuf* cb, RHIQueueType queue_type) = 0;
	virtual bool BeginFrame() = 0;
	virtual bool Present() = 0;
	virtual bool EndFrame() = 0;
	virtual ~IRHIDevice() = 0 {};
};

struct rhi {

	struct RenderContext {
		graphics::RenderWindowHandle rw_handle_;
		void* platform_data_;
	};
	typedef RenderContext*   RenderContextHandle;

	bool (*initialize_rhi)(graphics::RenderWindowHandle window);
	bool (*finalize_rhi)();
	unsigned int (*get_window_flags)(void);

	bool (*begin_frame)();
	bool (*end_frame)();

	IRHIDevice* (*create_device)(void);
	void(*destroy_device)(IRHIDevice* device);

	void (*make_current_context)(void);
};

