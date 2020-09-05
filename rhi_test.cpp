#include "engine/utils/timing.h"
#include "engine/utils/gl_utils.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/camera.h"
#include "engine/utils/frustum.h"
#include "engine/utils/obj_loader.h"
#include "engine/utils/macros.h"
#include "engine/gameos.hpp"

#include "rhi/rhi.h"

#include "scene.h"
#include "editor.h"
#include "res_man.h"
#include "renderer.h"
#include "shadow_renderer.h"
#include "particle_system.h"
#include "debug_renderer.h"
#include "deferred_renderer.h"
#include "obj_id_renderer.h"

#include "Remotery/lib/Remotery.h"
#include <cstdlib>
#include <cstddef>
#include <string>
#include <functional>
#include <list>
#include <unordered_map>

extern int RendererGetNumBufferedFrames();
extern int RendererGetCurrentFrame();
extern int GetCurrentFrame();
extern void SetRenderFrameContext(void* rfc);
extern void* GetRenderFrameContext();

//#define DO_TESTS
#if defined(DO_TESTS)
extern void test_fixed_block_allocator();
#endif

bool g_update_simulation = true;
uint32_t g_obj_under_cursor = scene::kInvalidObjectId;

camera g_camera;
camera g_shadow_camera;

#define SCREEN_W 1280.0f
#define SCREEN_H 900.0f

extern IRHIDevice* rhi_get_device();

void __stdcall Init(void)
{
    printf("::Init\n");

#if defined(DO_TESTS)
    test_fixed_block_allocator();
#endif


	const vec3 init_cam_pos(0, 15, 0);

    g_camera.set_pos(init_cam_pos);
    g_camera.set_projection(45.0f, (int)SCREEN_W, (int)SCREEN_H, 0.1f, 1000.0f);
    g_camera.update(0.0f);

    vec3 light_dir = normalize(vec3(-1.0,-1.0,-1.0));

    vec3 up = vec3(0,1,0);

    vec3 right = normalize(cross(up, light_dir));
    up = normalize(cross(light_dir, right));

    mat4 shadow_view = mat4::identity();
    // negative light dir by convention
    camera::compose_view_matrix(&shadow_view, right, up, -light_dir,
                                -light_dir * 50.0f);

    g_shadow_camera.set_ortho_projection(-30, 30, 30, -30, 0.1f, 200.0f);
    g_shadow_camera.set_view(shadow_view);

}

void __stdcall Deinit(void)
{
    DeleteRenderLists();

    finalize_editor();
    finalize_scene();

    printf("::Deinit\n");
}

void UpdateCamera(float dt)
{
    static float moveSpeedK = 50.0f;
    static float angularSpeedK = 0.25f * 3.1415f / 180.0f; // 0.25 degree per pixel

    g_camera.dx += gos_GetKeyStatus(KEY_D) ? dt*moveSpeedK : 0.0f;
    g_camera.dx -= gos_GetKeyStatus(KEY_A) ? dt*moveSpeedK : 0.0f;
    g_camera.dz += gos_GetKeyStatus(KEY_W) ? dt*moveSpeedK : 0.0f;
    g_camera.dz -= gos_GetKeyStatus(KEY_S) ? dt*moveSpeedK : 0.0f;

    int XDelta, YDelta, WheelDelta;
    float XPos, YPos;
    DWORD buttonsPressed;
    gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);

    g_camera.rot_x += (float)XDelta * angularSpeedK;
    g_camera.rot_y -= (float)YDelta * angularSpeedK;

    static float fov = 45.0f;
    fov += (float)WheelDelta;

    g_camera.set_projection(fov, Environment.drawableWidth, Environment.drawableHeight, 0.1f, 1000.0f);
    g_camera.update(dt);

}

void __stdcall Update(void)
{
    RenderFrameContext* rfc = new RenderFrameContext();

    static bool initialization_done = false;
    if(!initialization_done)
    {
        gos_SetRelativeMouseMode(true);
        initialization_done = true;
    }

    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    float dt_sec = ((float)timing::ticks2ms(end_tick - start_tick))/1e3f;
    start_tick = timing::gettickcount();

    if(gos_GetKeyStatus(KEY_P) == KEY_PRESSED)
        g_update_simulation = !g_update_simulation;

	UpdateCamera(dt_sec);

	g_obj_under_cursor = scene::kInvalidObjectId;

	// prepare list of objects to render
    RenderList* frame_render_list = AcquireRenderList();

    // setup render frame context
    CSMInfo csm_info;
    fill_csm_frustums(&csm_info, &g_camera, &g_shadow_camera);
    rfc->frame_number_ = GetCurrentFrame();
    rfc->rl_ = frame_render_list;
    rfc->csm_info_ = csm_info;
	rfc->inv_view_ = g_camera.get_inv_view();
    rfc->view_ = g_camera.get_view();
    rfc->proj_ = g_camera.get_projection();
    rfc->inv_proj_ = g_camera.get_inv_projection();
    g_shadow_camera.get_view(&rfc->shadow_view_);
    rfc->shadow_inv_view_ = g_shadow_camera.get_inv_view();
    SetRenderFrameContext(rfc);
    //
    
    //uint64_t sleep_ms= std::max(33ull - dt, 1ull);
    //timing::sleep(sleep_ms*1000000);
    timing::sleep(32000000ull);
}

IRHICmdBuf* g_cmdbuf[8] = { nullptr };
IRHIRenderPass* g_main_pass = nullptr;
std::vector<IRHIFrameBuffer*> g_main_fb;

void __stdcall Render(void)
{
    static bool initialized = false;
    // should this be a command added by Update to render thread?
    if(!initialized)
    {
		IRHIDevice* device = rhi_get_device();
		for (int i = 0; i < countof(g_cmdbuf); ++i) {
			g_cmdbuf[i] = device->CreateCommandBuffer(RHIQueueType::kPresentation);
		}

		RHIAttachmentDesc att_desc;
		att_desc.format = device->GetSwapChainFormat();
		att_desc.numSamples = 1;
		att_desc.loadOp = RHIAttachmentLoadOp::kClear;
		att_desc.storeOp = RHIAttachmentStoreOp::kStore;
		att_desc.stencilLoadOp = RHIAttachmentLoadOp::kDoNotCare;
		att_desc.stencilStoreOp = RHIAttachmentStoreOp::kDoNotCare;
		att_desc.initialLayout = RHIImageLayout::kUndefined;
		att_desc.finalLayout = RHIImageLayout::kPresent;

		RHIAttachmentRef color_att_ref = { 0, RHIImageLayout::kColorOptimal };

		RHISubpassDesc sp_desc;
		sp_desc.bindPoint = RHIPipelineBindPoint::kGraphics;
		sp_desc.colorAttachmentCount = 1;
		sp_desc.colorAttachments = &color_att_ref;
		sp_desc.depthStencilAttachmentCount = 0;
		sp_desc.depthStencilAttachments = nullptr;
		sp_desc.inputAttachmentCount = 0;
		sp_desc.inputAttachments = nullptr;
		sp_desc.preserveAttachmentCount = 0;
		sp_desc.preserveAttachments = nullptr;

		RHIRenderPassDesc rp_desc;
		rp_desc.attachmentCount = 1;
		rp_desc.attachmentDesc = &att_desc;
		rp_desc.subpassCount = 1;
		rp_desc.subpassDesc = &sp_desc;
		rp_desc.dependencyCount = 0;
		rp_desc.dependencies = nullptr;

		g_main_pass = device->CreateRenderPass(&rp_desc);

		g_main_fb.resize(device->GetSwapChainSize());
		for (int i = 0; i < g_main_fb.size(); ++i) {
			const IRHIImageView* view = device->GetSwapChainImageView(i);
			const IRHIImage* image = device->GetSwapChainImage(i);

			RHIFrameBufferDesc fb_desc;
			fb_desc.attachmentCount = 1;
			fb_desc.pAttachments = &view;
			fb_desc.width_ = image->Width();
			fb_desc.height_= image->Height();
			fb_desc.layers_ = 1;
			g_main_fb[i] = device->CreateFrameBuffer(&fb_desc, g_main_pass);
		}


		//TODO: add render thread destroy callback to destroy all render resources


        initialized = true;
    }

    RenderFrameContext* rfc_nonconst_because_of_partiles = (RenderFrameContext*)GetRenderFrameContext();
    assert(rfc_nonconst_because_of_partiles);
    assert(rfc_nonconst_because_of_partiles->frame_number_ == RendererGetCurrentFrame());

    // add particle systems tasks to render list
    RenderFrameContext* rfc = rfc_nonconst_because_of_partiles;
	(void)rfc;
    // process all scheduled commands
    for(auto& cmd : rfc->commands_) {
        cmd();
    }
    rfc->commands_.clear();

	IRHIDevice* device = rhi_get_device();
	IRHIImage* fb_image = device->GetCurrentSwapChainImage();

	static int cur_idx = 0;
	IRHICmdBuf* cb = g_cmdbuf[cur_idx];

	static vec4 color = vec4(1, 0, 0, 0);

	cb->Begin();
	cb->Barrier_PresentToClear(fb_image);
	cb->Clear(fb_image, color, (uint32_t)RHIImageAspectFlags::kColor);
	cb->Barrier_ClearToPresent(fb_image);
	cb->End();
	device->Submit(cb, RHIQueueType::kPresentation);

	cur_idx = (cur_idx + 1) % ((int)countof(g_cmdbuf));

	uint64_t ticks = timing::gettickcount();
    float sec = ((float)timing::ticks2ms(ticks))/1e3f;

	color.x = 0.5f*sin(2 * 3.1415f*sec) + 0.5f;

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
	(void)rpl;
    ReleaseRenderList(rfc->rl_);
    delete rfc;

}

void GetGameOSEnvironment(const char* cmdline)
{
    (void)cmdline;
    Environment.screenWidth = (int)SCREEN_W;
    Environment.screenHeight = (int)SCREEN_H;

    Environment.InitializeGameEngine = Init;
    Environment.DoGameLogic = Update;
    Environment.TerminateGameEngine = Deinit;
    Environment.UpdateRenderers = Render;
}




