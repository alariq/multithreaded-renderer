#pragma once

#include "gos_render.h"
#include "utils/vec.h"
#include <stdint.h>

class IRHIImageView;
class IRHIImage;

enum class RHIQueueType: uint32_t {
	kUnknown = 0x0,
	kGraphics = 0x1,
	kCompute = 0x2,
	kPresentation = 0x4,
};

enum class RHIFormat: uint32_t {
	kR8G8B8A8_UNORM = 0,
	kR8G8B8A8_UINT = 1,
	kR8G8B8A8_SRGB = 2,

	kR32_UINT = 3,
    kR32_SINT = 4,
	kR32_SFLOAT = 5,

    kR32G32_UINT = 6,
    kR32G32_SINT = 7,
    kR32G32_SFLOAT = 8,

    kR32G32B32_UINT = 9,
    kR32G32B32_SINT = 10,
    kR32G32B32_SFLOAT = 11,

    kR32G32B32A32_UINT = 12,
    kR32G32B32A32_SINT = 13,
    kR32G32B32A32_SFLOAT = 14,
};

enum class RHIPipelineStageFlags: uint32_t {
	kTopOfPipe = 0x00000001,
	kDrawIndirect = 0x00000002,
	kVertexInput = 0x00000004,
	kVertexShader = 0x00000008,
	kTessellationControlShader = 0x00000010,
	kTessellationEvaluationShader = 0x00000020,
	kGeometryShader = 0x00000040,
	kFragmentShader = 0x00000080,
	kEarlyFragmentTests = 0x00000100,
	kLateFragmentTests = 0x00000200,
	kColorAttachmentOutput = 0x00000400,
	kComputeShader = 0x00000800,
	kTransfer = 0x00001000,
	kBottomOfPipe = 0x00002000,
	kHost = 0x00004000,
	kAllGraphics = 0x00008000,
	kAllCommands = 0x00010000,
};

enum class RHIAccessFlags: uint32_t {
	kIndirectCommandRead = 0x00000001,
	kIndexRead = 0x00000002,
	kVertexAttributeRead = 0x00000004,
	kUniformRead = 0x00000008,
	kInputAttachmentRead = 0x00000010,
	kShaderRead = 0x00000020,
	kShaderWrite = 0x00000040,
	kColorAttachmentRead = 0x00000080,
	kColorAttachmentWrite = 0x00000100,
	kDepthStencilAttachmentRead = 0x00000200,
	kDepthStencilAttachmentWrite = 0x00000400,
	kTransferRead = 0x00000800,
	kTransferWrite = 0x00001000,
	kHostRead = 0x00002000,
	kHostWrite = 0x00004000,
	kMemoryRead = 0x00008000,
	kMemoryWrite = 0x00010000,
};

enum class RHIDependencyFlags {
    kByRegion = 0x00000001,
    kDeviceGroup = 0x00000004,
    kViewLocal = 0x00000002,
};

enum class RHIImageAspectFlags : uint32_t {
	kColor = 0x1,
	kDepth = 0x2,
	kStencil = 0x4
};

enum class RHIImageLayout : uint32_t {
	kUndefined = 0,
	kGeneral = 1,
	kColorOptimal = 2,
    kDepthStencilOptimal = 3,
    kDepthStencilReadOnlyOptimal = 4,
    kShaderReadOnlyOptimal = 5,
    kTransferSrcOptimal = 6,
    kTransferDstOptimal = 7,
	kPresent = 8,
};

enum class RHIAttachmentStoreOp : uint32_t {
	kStore = 0,
	kDoNotCare = 1,
};
enum class RHIAttachmentLoadOp : uint32_t {
	kLoad = 0,
	kClear = 1,
	kDoNotCare = 2,
};

enum class RHIPipelineBindPoint : uint32_t {
	kGraphics = 0,
	kCompute = 1,
	kRayTracing = 2,
};

enum class RHIImageViewType: uint32_t {
    k1d = 0,
    k2d = 1,
    k3d = 2,
    kCube = 3,
    k1dArray = 4,
    k2dArray = 5,
    kCubeArray = 6,
};

enum class RHIPrimitiveTopology: uint32_t {
    kPointList = 0,
    kLineList = 1,
    kLineStrip = 2,
    kTriangleList = 3,
    kTriangleStrip = 4,
    kTriangleFan = 5,
    kLineListWithAdjacency = 6,
    kLineStripWithAdjacency = 7,
    kTriangleListWithAdjacency = 8,
    kTriangleStripWithAdjacency = 9,
    kPatchList = 10,
};

enum class RHIPolygonMode: uint32_t {
    kFill = 0,
    kLine = 1,
    kPoint = 2,
};

enum class RHICullModeFlags: uint32_t {
    kNone = 0,
    kFront = 0x00000001,
    kBack = 0x00000002,
    kFrontAndBack = 0x00000003,
};

enum RHIFrontFace: uint32_t {
    kCounterClockwise = 0,
    kClockwise = 1
};

enum class RHICompareOp: uint32_t {
	kNever = 0,
	kLess = 1,
	kEqual = 2,
	kLessOrEqual = 3,
	kGreater = 4,
	kNotEqual = 5,
	kGreaterOrEqual = 6,
	kAlways = 7,
};

enum class RHIStencilOp: uint32_t {
    kKeep = 0,
    kZero = 1,
    kReplace = 2,
    kIncrementAndClamp = 3,
    kDecrementAndClamp = 4,
    kInvert = 5,
    kIncrementAndWrap = 6,
    kDecrementAndWrap = 7,
};

enum RHILogicOp: uint32_t {
    kClear = 0,
    kAnd = 1,
    kAndReverse = 2,
    kCopy = 3,
    kAndInverted = 4,
    kNoOp = 5,
    kXor = 6,
    kOr = 7,
    kNor = 8,
    kEquivalent = 9,
    kInvert = 10,
    kOrReverse = 11,
    kCopyInverted = 12,
    kOrInverted = 13,
    kNand = 14,
    kSet = 15,
};

enum class RHIBlendFactor: uint32_t {
    Zero = 0,
    One = 1,
    SrcColor = 2,
    OneMinusSrcColor = 3,
    DstColor = 4,
    OneMinusDstColor = 5,
    SrcAlpha = 6,
    OneMinusSrcAlpha = 7,
    DstAlpha = 8,
    OneMinusDstAlpha = 9,
};

enum class RHIBlendOp: uint32_t {
    kAdd = 0,
    kSubtract = 1,
    kReverseSubtract = 2,
    kMin = 3,
    kMax = 4,
};

enum class RHIColorComponentFlags: uint32_t {
    kR = 0x00000001,
    kG = 0x00000002,
    kB = 0x00000004,
    kA = 0x00000008,
};

////////////////////////////////////////////////////////////////////////////////
struct RHIImageSubresourceRange {
    uint32_t			aspectMask;
    uint32_t            baseMipLevel;
    uint32_t            levelCount;
    uint32_t            baseArrayLayer;
    uint32_t            layerCount;
};
	
////////////////////////////////////////////////////////////////////////////////
struct RHIImageViewDesc {
	IRHIImage*                    image;
    RHIImageViewType            viewType;
    RHIFormat                   format;
    //RHIComponentMapping         components;
    RHIImageSubresourceRange    subresourceRange;
};

////////////////////////////////////////////////////////////////////////////////
struct RHIFrameBufferDesc {
    uint32_t                    attachmentCount;
    const IRHIImageView* const*	pAttachments;
    uint32_t                    width_;
    uint32_t                    height_;
    uint32_t                    layers_;
	RHIFrameBufferDesc& width(uint32_t w) { width_ = w; return *this; }
	RHIFrameBufferDesc& height(uint32_t h) { height_ = h; return *this; }
	RHIFrameBufferDesc& layers(uint32_t l) { layers_ = l; return *this; }
};

////////////////////////////////////////////////////////////////////////////////
enum class RHIShaderStageFlags: uint32_t {
	kVertex = 0x00000001,
	kTessellationControl = 0x00000002,
	kTessellationEvaluation = 0x00000004,
	kGeometry = 0x00000008,
	kFragment = 0x00000010,
	kCompute = 0x00000020,
};

////////////////////////////////////////////////////////////////////////////////
struct RHIAttachmentRef {
	int index;
	RHIImageLayout layout;
};
struct RHIAttachmentDesc {
    RHIFormat                        format;
    uint32_t						numSamples;
    RHIAttachmentLoadOp              loadOp;
    RHIAttachmentStoreOp             storeOp;
    RHIAttachmentLoadOp              stencilLoadOp;
    RHIAttachmentStoreOp             stencilStoreOp;
    RHIImageLayout                   initialLayout;
    RHIImageLayout                   finalLayout;
};

////////////////////////////////////////////////////////////////////////////////
struct RHISubpassDesc {
	RHIPipelineBindPoint bindPoint;
	int inputAttachmentCount;
	RHIAttachmentRef* inputAttachments;
	int colorAttachmentCount;
	RHIAttachmentRef* colorAttachments;
	int depthStencilAttachmentCount;
	RHIAttachmentRef* depthStencilAttachments;
	int preserveAttachmentCount;
	uint32_t* preserveAttachments;
};

struct RHISubpassDependency {
    uint32_t         srcSubpass;
    uint32_t         dstSubpass;
    RHIPipelineStageFlags srcStageMask;
    RHIPipelineStageFlags dstStageMask;
    RHIAccessFlags   srcAccessMask;
    RHIAccessFlags  dstAccessMask;
    RHIDependencyFlags	dependencyFlags;
};

struct RHIRenderPassDesc {
	int attachmentCount;
	RHIAttachmentDesc* attachmentDesc;
	int subpassCount;
	RHISubpassDesc* subpassDesc;
	int dependencyCount;
	RHISubpassDependency* dependencies;
};
////////////////////////////////////////////////////////////////////////////////

enum class RHIVertexInputRate: uint32_t {
    kVertex = 0,
    kInstance = 1,
};

struct RHIVertexInputBindingDesc {
    uint32_t            binding;
    uint32_t            stride;
    RHIVertexInputRate	inputRate;
};

struct RHIVertexInputAttributeDesc {
    uint32_t    location;
    uint32_t    binding;
    RHIFormat    format;
    uint32_t    offset;
};

////////////////////////////////////////////////////////////////////////////////
struct RHIViewport {
	float x;
	float y;
	float width;
	float height;
	float minDepth;
	float maxDepth;
};

struct RHIScissor {
	int32_t x;
	int32_t y;
	int32_t width;
	int32_t height;
};

struct RHIShaderStage {
	RHIShaderStageFlags stage;
	class IRHIShader *module;
	const char *pEntryPointName;
};

struct RHIVertexInputState {
	uint32_t vertexBindingDescCount;
	const RHIVertexInputBindingDesc *pVertexBindingDesc;
	uint32_t vertexAttributeDescCount;
	const RHIVertexInputAttributeDesc *pVertexAttributeDesc;
};

struct RHIInputAssemblyState {
    RHIPrimitiveTopology topology;
    bool primitiveRestartEnable;
};

struct RHIViewportState {
    uint32_t                              viewportCount;
    const RHIViewport*                     pViewports;
    uint32_t                              scissorCount;
    const RHIScissor*                       pScissors;
};

struct RHIRasterizationState {
	bool depthClampEnable;
	bool rasterizerDiscardEnable;
	RHIPolygonMode polygonMode;
	RHICullModeFlags cullMode;
	RHIFrontFace frontFace;
	bool depthBiasEnable;
	float depthBiasConstantFactor;
	float depthBiasClamp;
	float depthBiasSlopeFactor;
	float lineWidth;
};

struct RHIMultisampleState {
	uint32_t rasterizationSamples;
	bool sampleShadingEnable;
	float minSampleShading;
	const uint32_t *pSampleMask;
	bool alphaToCoverageEnable;
	bool alphaToOneEnable;
};

struct RHIColorBlendAttachmentState {
    uint32_t				  blendEnable;
    RHIBlendFactor            srcColorBlendFactor;
    RHIBlendFactor            dstColorBlendFactor;
    RHIBlendOp                colorBlendOp;
    RHIBlendFactor            srcAlphaBlendFactor;
    RHIBlendFactor            dstAlphaBlendFactor;
    RHIBlendOp                alphaBlendOp;
    RHIColorComponentFlags    colorWriteMask;
};

struct RHIColorBlendState {
	bool logicOpEnable;
	RHILogicOp logicOp;
	uint32_t attachmentCount;
	const RHIColorBlendAttachmentState *pAttachments;
	float blendConstants[4];
};

////////////////////////////////////////////////////////////////////////////////

class IRHIImage {
protected:
	RHIFormat format_;
	uint32_t width_;
	uint32_t height_;
public:
	RHIFormat Format() const { return format_; }
	uint32_t Width() const { return width_; }
	uint32_t Height() const { return width_; }

};

class IRHIImageView {
protected:
	//...
	IRHIImage* image_; // ? needed
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
	virtual ~IRHICmdBuf() = 0;
};

class IRHIRenderPass {
public:
	virtual ~IRHIRenderPass() = 0;
};

class IRHIFrameBuffer {
public:
	virtual ~IRHIFrameBuffer() = 0;
};

class IRHIShader {
public:
	virtual ~IRHIShader() = 0;
};

class IRHIDescriptorSetLayout {
public:
	virtual ~IRHIDescriptorSetLayout() = 0;
};

class IRHIPipelineLayout {
public:
	virtual ~IRHIPipelineLayout() = 0;
};

class IRHIGraphicsPipeline{
public:
	virtual ~IRHIGraphicsPipeline() = 0; 
};



class IRHIDevice {
public:

	virtual IRHICmdBuf*			CreateCommandBuffer(RHIQueueType queue_type) = 0;
	virtual IRHIRenderPass*		CreateRenderPass(const RHIRenderPassDesc* desc) = 0;
	virtual IRHIFrameBuffer*	CreateFrameBuffer(const RHIFrameBufferDesc* desc, const IRHIRenderPass* rp_in) = 0;
	virtual IRHIImageView*		CreateImageView(const RHIImageViewDesc* desc) = 0;

	virtual RHIFormat				GetSwapChainFormat() = 0;
	virtual uint32_t				GetSwapChainSize() = 0;
	virtual const IRHIImageView*	GetSwapChainImageView(uint32_t index) = 0;
	virtual const class IRHIImage*	GetSwapChainImage(uint32_t index) = 0;
	virtual IRHIImage*				GetCurrentSwapChainImage() = 0;

	virtual bool Submit(IRHICmdBuf* cb, RHIQueueType queue_type) = 0;
	virtual bool BeginFrame() = 0;
	virtual bool Present() = 0;
	virtual bool EndFrame() = 0;
	virtual ~IRHIDevice() {};
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

