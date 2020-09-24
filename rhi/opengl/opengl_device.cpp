#include "rhi.h"
#include "GL/glew.h"
#include "opengl_device.h"
#include "utils/logging.h"
#include "utils/macros.h"
#include <unordered_map>
#include <cassert>
#include <memory>

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
	GLuint rv = (bits & (uint32_t)RHIImageAspectFlags::kColor) ? GL_COLOR_BUFFER_BIT: 0;
	rv |= (bits & (uint32_t)RHIImageAspectFlags::kDepth) ? GL_DEPTH_BUFFER_BIT: 0;
	rv |= (bits & (uint32_t)RHIImageAspectFlags::kStencil) ? GL_STENCIL_BUFFER_BIT: 0;
	return rv;
}

////////////// Image //////////////////////////////////////////////////


////////////// Shader //////////////////////////////////////////////////
void RHIShaderGL::Destroy(IRHIDevice* device) {
	delete this;
}

RHIShaderGL *RHIShaderGL::Create(IRHIDevice *device, const uint32_t *pdata, uint32_t size,
								 RHIShaderStageFlags stage) {
	RHIShaderGL *shader = new RHIShaderGL();
	shader->stage_ = stage;
	return shader;
}

////////////// Pipeline Layout //////////////////////////////////////////////////
void RHIPipelineLayoutGL::Destroy(IRHIDevice* device) {
	delete this;
}

RHIPipelineLayoutGL *RHIPipelineLayoutGL::Create(IRHIDevice *device,
												 IRHIDescriptorSetLayout *desc_set_layout,
												 RHIShaderStageFlags stage) {
	RHIPipelineLayoutGL *pl = new RHIPipelineLayoutGL();
	return pl;
}

////////////// Graphics Pipeline //////////////////////////////////////////////////
void RHIGraphicsPipelineGL::Destroy(IRHIDevice* device) {
	delete this;
}

RHIGraphicsPipelineGL *RHIGraphicsPipelineGL::Create(
		IRHIDevice *device, RHIShaderStage *shader_stage, uint32_t shader_stage_count,
		RHIVertexInputState *vertex_input_state, RHIInputAssemblyState* input_assembly_state,
		RHIViewportState* viewport_state, RHIRasterizationState *raster_state,
		RHIMultisampleState* multisample_state, RHIColorBlendState* color_blend_state) {

	return nullptr;
}

////////////// Buffer //////////////////////////////////////////////////

void RHIBufferGL::Destroy(IRHIDevice* device) {
	delete this;
}

void *RHIBufferGL::Map(IRHIDevice* device, uint32_t offset, uint32_t size, uint32_t flags) {
    assert(!is_mapped_);
    assert(mem_flags & RHIMemoryPropertyFlags::kHostVisible);

    is_mapped_ = true;
    mapped_offset_ = offset;
    mapped_size_ = size;
    mapped_flags_ = flags;

    return nullptr;
}

void RHIBufferGL::Unmap(IRHIDevice* device) {
    assert(is_mapped_);
    is_mapped_ = false;
}

////////////// Command buffer //////////////////////////////////////////////////

bool RHICmdBufGL::Begin() {
	is_recording_ = true;
	return true;
}

bool RHICmdBufGL::BeginRenderPass(IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb, const ivec4 *render_area,
						const RHIClearValue *clear_values, uint32_t count) {
    assert(is_recording_);
    return true;
}

bool RHICmdBufGL::End() {
	is_recording_ = false;
	return true;
}

void RHICmdBufGL::EndRenderPass(const IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb) {
    assert(is_recording_);
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

void RHICmdBufGL::Barrier_ClearToPresent(IRHIImage *) {
}
void RHICmdBufGL::Barrier_PresentToClear(IRHIImage *) {
}
void RHICmdBufGL::Barrier_PresentToDraw(IRHIImage *) {
}
void RHICmdBufGL::Barrier_DrawToPresent(IRHIImage* image) {
}

void RHICmdBufGL::BindPipeline(RHIPipelineBindPoint bind_point, IRHIGraphicsPipeline* pipeline) {
}

void RHICmdBufGL::BindVertexBuffers(IRHIBuffer** i_vb, uint32_t first_binding, uint32_t count) {
}

////////////////RHI Device /////////////////////////////////////////////////////

RHIDeviceGL::RHIDeviceGL() {
	fb_ = std::make_unique<RHIImageGL>(0, 1024u, 768u);
	// TODO: set w/h/format
	// need to pass window info here
	fb_view_ = std::make_unique<RHIImageViewGL>(0);
}

RHIDeviceGL::~RHIDeviceGL() {
}

IRHIFrameBuffer* RHIDeviceGL::CreateFrameBuffer(RHIFrameBufferDesc* , const IRHIRenderPass* ) {
	return new RHIFrameBufferGL();
}

IRHIRenderPass* RHIDeviceGL::CreateRenderPass(const RHIRenderPassDesc* ) {
	return new RHIRenderPassGL();
}

IRHIImageView* RHIDeviceGL::CreateImageView(const RHIImageViewDesc* ) {
	return new RHIImageViewGL(0);
}

IRHIBuffer* CreateBuffer(uint32_t size, uint32_t usage, uint32_t memprop, RHISharingMode sharing) {
	return new RHIBufferGL(0);
}

// should this be mobved to command buffer class / .cpp file and just pass Device as a parameter ?
IRHICmdBuf* RHIDeviceGL::CreateCommandBuffer(RHIQueueType queue_type) {

	assert(queue_type == RHIQueueType::kGraphics || queue_type == RHIQueueType::kPresentation);
	return new RHICmdBufGL();

}

IRHIGraphicsPipeline *RHIDeviceGL::CreateGraphicsPipeline(
            const RHIShaderStage *shader_stage, uint32_t shader_stage_count,
            const RHIVertexInputState *vertex_input_state,
            const RHIInputAssemblyState *input_assembly_state, const RHIViewportState *viewport_state,
            const RHIRasterizationState *raster_state, const RHIMultisampleState *multisample_state,
            const RHIColorBlendState *color_blend_state, const IRHIPipelineLayout *i_pipleline_layout,
            const IRHIRenderPass *i_render_pass) {
    return nullptr;
}

IRHIPipelineLayout* RHIDeviceGL::CreatePipelineLayout(IRHIDescriptorSetLayout* desc_set_layout) {
    return nullptr;
}

IRHIShader* RHIDeviceGL::CreateShader(RHIShaderStageFlags stage, const uint32_t *pdata, uint32_t size) {
    return nullptr;
}

bool RHIDeviceGL::BeginFrame() {

	cur_frame_++;
	between_begin_frame = true;

	return true;
}

bool RHIDeviceGL::Submit(IRHICmdBuf* cb_in, RHIQueueType ) {

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
