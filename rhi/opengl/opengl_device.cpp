#include "rhi.h"
#include "gl/glew.h"
#include "opengl_device.h"
#include "utils/logging.h"
#include "utils/macros.h"
#include <unordered_map>
#include <cassert>

////////////////////////////////////////////////////////////////////////////////
template <typename T> struct ResImplType;
template<> struct ResImplType<IRHICmdBuf> { typedef RHICmdBufGL Type; };
template<> struct ResImplType<IRHIImage> { typedef RHIImageGL Type; };
template<> struct ResImplType<IRHIDevice> { typedef RHIDeviceGL Type; };

template <typename R> 
typename ResImplType<R>::Type* ResourceCast(R* obj) {
	return static_cast<typename ResImplType<R>::Type*>(obj);
}
////////////////////////////////////////////////////////////////////////////////
GLuint translate_image_aspect(uint32_t bits) {
	GLuint rv = (bits & (uint32_t)RHIImageAspect::kColor) ? GL_COLOR_BUFFER_BIT: 0;
	rv |= (bits & (uint32_t)RHIImageAspect::kDepth) ? GL_DEPTH_BUFFER_BIT: 0;
	rv |= (bits & (uint32_t)RHIImageAspect::kStencil) ? GL_STENCIL_BUFFER_BIT: 0;
	return rv;
}

////////////// Image //////////////////////////////////////////////////


////////////// Command buffer //////////////////////////////////////////////////

bool RHICmdBufGL::Begin() {
	is_recording_ = true;
	return true;
}

bool RHICmdBufGL::End() {
	is_recording_ = false;
	return true;
}

void RHICmdBufGL::Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) {
	assert(is_recording_);
	RHIImageGL* image = ResourceCast(image_in);
	(void)image;
	assert(image->Handle() == 0);
	GLuint clear_bits = translate_image_aspect(img_aspect_bits);

	glClearColor(color.x, color.y, color.z, color.w);
	glClear(clear_bits);
	
}

void RHICmdBufGL::Barrier_ClearToPresent(IRHIImage *image_in) {
}
void RHICmdBufGL::Barrier_PresentToClear(IRHIImage *image_in) {
}

////////////////RHI Device /////////////////////////////////////////////////////

RHIDeviceGL::~RHIDeviceGL() {
}


// should this be mobved to command buffer class / .cpp file and just pass Device as a parameter ?
IRHICmdBuf* RHIDeviceGL::CreateCommandBuffer(RHIQueueType queue_type) {

	assert(queue_type == RHIQueueType::kGraphics || queue_type == RHIQueueType::kPresentation);
	return new RHICmdBufGL();

}

bool RHIDeviceGL::BeginFrame() {

	cur_frame_++;
	between_begin_frame = true;
	fb_.SetImage({ 0 });

	return true;
}

bool RHIDeviceGL::Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) {

	RHICmdBufGL* cb = ResourceCast(cb_in);
	(void)(cb);

	return true;
}

bool RHIDeviceGL::Present() {
	return true;
}

bool RHIDeviceGL::EndFrame() {
	assert(prev_frame_ == cur_frame_ - 1);
	prev_frame_ = cur_frame_;
	between_begin_frame = false;
	return true;
}
