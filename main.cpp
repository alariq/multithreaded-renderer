#include "engine/utils/timing.h"
#include "engine/utils/gl_utils.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/camera.h"
#include "engine/utils/frustum.h"
#include "engine/utils/obj_loader.h"
#include "engine/gameos.hpp"

#include "sph.h"
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

bool g_is_in_editor = false;
bool g_render_initialized_hack = false;
bool g_update_simulation = false;
uint32_t g_obj_under_cursor = scene::kInvalidObjectId;

DWORD g_htexture = 0;
ShadowRenderPass* g_shadow_pass = nullptr;
uint32_t g_show_cascade_index = 2;
bool render_from_shadow_camera = false;

DeferredRenderer g_deferred_renderer;
ObjIdRenderer g_obj_id_renderer;

camera g_camera;
camera g_shadow_camera;

#define SCREEN_W 1280.0f
#define SCREEN_H 900.0f

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
    g_obj_id_renderer.Deinit();

    DeleteRenderLists();

    delete g_shadow_pass;

    finalize_editor();
    finalize_scene();
    finalize_res_man();

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

    render_from_shadow_camera = gos_GetKeyStatus(KEY_O) ? true : false;
    g_show_cascade_index += gos_GetKeyStatus(KEY_C) == KEY_PRESSED ? 1 : 0;
    if(g_show_cascade_index>2)
        g_show_cascade_index = 0;

}

void __stdcall Update(void)
{
    RenderFrameContext* rfc = new RenderFrameContext();

    static bool initialization_done = false;
    if(!initialization_done)
    {
        gos_SetRelativeMouseMode(true);
        initialize_scene(&g_camera, rfc);
        initialize_editor();
        initialization_done = true;
    }

    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    float dt_sec = ((float)timing::ticks2ms(end_tick - start_tick))/1e3f;

    //printf("dt: %zd\n", dt);

    start_tick = timing::gettickcount();

    if(gos_GetKeyStatus(KEY_P) == KEY_PRESSED)
        g_update_simulation = !g_update_simulation;

    if(gos_GetKeyStatus(KEY_TAB) == KEY_PRESSED)
    {
        g_is_in_editor = !g_is_in_editor;
        gos_SetRelativeMouseMode(!g_is_in_editor);
    }

	scene_set_object_id_under_cursor(g_obj_under_cursor);

	if (!g_is_in_editor) {
		// game update
		UpdateCamera(dt_sec);
	} else {
		editor_update(&g_camera, dt_sec);
	}

	if (g_update_simulation)
        scene_update(&g_camera, dt_sec);

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

    char listidx_str[32] = {0};
    sprintf(listidx_str, "list_idx: %d", frame_render_list->GetId());
    rmt_BeginCPUSampleDynamic(listidx_str, 0);

    if(g_render_initialized_hack)
    {
		scene_render_update(rfc);
		if(g_is_in_editor)
			editor_render_update(rfc);
	}

    rmt_EndCPUSample();

    
    //uint64_t sleep_ms= std::max(33ull - dt, 1ull);
    //timing::sleep(sleep_ms*1000000);
    timing::sleep(32000000ull);
}

class ShapeRenderer {

	mat4 view_;
	mat4 proj_;
    const DWORD* shadow_maps_;
    const mat4* shadow_matrices_;
    vec4 lightdir_;
    const float* zfar_;
    HGOSTEXTURESAMPLER shadow_sampler_;
    DWORD gos_default_texture_;

public:

	void setup(const mat4& view, const mat4& proj)
	{
		view_ = view;
        proj_ = proj;
	}

    void set_shadow_params(const vec4 lightdir, const mat4 *shadow_matrices,
                           const DWORD *shadow_maps,
                           const HGOSTEXTURESAMPLER shadow_sampler,
                           const float *zfar,
                           const DWORD gos_default_texture) {
        shadow_matrices_ = shadow_matrices;
        shadow_maps_ = shadow_maps;
        shadow_sampler_ = shadow_sampler;
        lightdir_ = lightdir;
        zfar_ = zfar;
        gos_default_texture_ = gos_default_texture;
    }

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");

        gos_SetRenderState(gos_State_Texture, ro.tex_id_ ? ro.tex_id_ : gos_default_texture_);
        gos_SetRenderState(gos_State_Filter, gos_FilterBiLinear);

        gos_SetRenderState(gos_State_ZCompare, 1);
        gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

        gos_SetRenderState(gos_State_Texture2, shadow_maps_[0]);
        gos_SetSamplerState(1, shadow_sampler_);
        gos_SetRenderState(gos_State_Texture3, shadow_maps_[1]);
        gos_SetSamplerState(2, shadow_sampler_);

        mat4 vp = proj_ * view_;
        mat4 wvp = vp * rp.m_;
        mat4 world_normal_;
        glu_InvertMatrixf(rp.m_, (float*)&world_normal_);
        world_normal_ = transpose(world_normal_);
        mat4 vpshadow0 = shadow_matrices_[0];
        mat4 vpshadow1 = shadow_matrices_[1];
        float params[] = {0.0f,
                          (shadow_maps_[0] || shadow_maps_[1]) ? 1.0f : 0.0f,
                           0.0f, 0.0f};

        //gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		gos_SetRenderMaterialParameterMat4(mat, "world_normal_", (const float*)world_normal_);
		//gos_SetRenderMaterialParameterMat4(mat, "vpshadow_", (const float*)vpshadow);
		gos_SetRenderMaterialParameterMat4(mat, "wvpshadow0_", (const float*)(vpshadow0 * rp.m_));
		gos_SetRenderMaterialParameterMat4(mat, "wvpshadow1_", (const float*)(vpshadow1 * rp.m_));
		gos_SetRenderMaterialParameterFloat4(mat, "lightdir_", (const float*)lightdir_);
		gos_SetRenderMaterialParameterFloat4(mat, "params_", params);
		gos_SetRenderMaterialParameterFloat4(mat, "z_far_", zfar_);

		gos_ApplyRenderMaterial(mat);

		// TODO: either use this or setMat4("wvp_", ...);
		//mat->setTransform(*wvp_);

        if (ro.ib_) {
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_, ro.prim_type_);
        } else if(ro.inst_vb_) {
    		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_, ro.prim_type_);
        } else {
    		gos_RenderArray(ro.vb_, ro.vdecl_, ro.prim_type_);
        }
    }
};

void render_quad(uint32_t tex_id, const vec4& scale_offset, HGOSRENDERMATERIAL mat_override)
{
	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    glViewport(0, 0, (GLsizei)Environment.drawableWidth, (GLsizei)Environment.drawableHeight);
    
    gos_SetRenderState(gos_State_Texture, tex_id);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);

    HGOSRENDERMATERIAL mat = mat_override ? mat_override : gos_getRenderMaterial(tex_id ? "textured_quad" : "coloured_quad");

    float colour[4] = {1.0f,1.0f,1.0f,1.0f};
    if(!tex_id)
    {
        gos_SetRenderMaterialParameterFloat4(mat, "colour", colour);
    }

    gos_SetRenderMaterialParameterFloat4(mat, "scale_offset", scale_offset);
    gos_ApplyRenderMaterial(mat);

    RenderMesh* fs_quad = res_man_load_mesh("fs_quad");
    gos_RenderIndexedArray(fs_quad->ib_, fs_quad->vb_, fs_quad->vdecl_, fs_quad->prim_type_);

}

void render_fullscreen_quad(uint32_t tex_id)
{
    render_quad(tex_id, vec4(1,1,0,0), nullptr);
}

void __stdcall Render(void)
{
    static bool initialized = false;
    // should this be a command added by Update to render thread?
    if(!initialized)
    {
        initialize_res_man();

        g_shadow_pass = new ShadowRenderPass();
        if(!g_shadow_pass->Init(1024, 2))
        {
            SPEW(("SHADOWS", "Failed to init shadow pass"));
        }

        gos_AddRenderMaterial("directional_shadow");
        gos_AddRenderMaterial("particle");
        sph_init_renderer();

		uint32_t w = (uint32_t)SCREEN_W;
		uint32_t h = (uint32_t)SCREEN_H;
        g_deferred_renderer.Init(w, h);
        g_obj_id_renderer.Init(w, h);

        initialized = true;

        g_render_initialized_hack = true;
    }

    ParticleSystemManager::Instance().InitRenderResources();

    RenderFrameContext* rfc_nonconst_because_of_partiles = (RenderFrameContext*)GetRenderFrameContext();
    assert(rfc_nonconst_because_of_partiles);
    assert(rfc_nonconst_because_of_partiles->frame_number_ == RendererGetCurrentFrame());

    // add particle systems tasks to render list
    ParticleSystemManager::Instance().Render(rfc_nonconst_because_of_partiles);
    RenderFrameContext* rfc = rfc_nonconst_because_of_partiles;

    // process all scheduled commands
    for(auto& cmd : rfc->commands_) {
        cmd();
    }
    rfc->commands_.clear();

    const CSMInfo& csm_info = rfc->csm_info_;

    // render shadows first
    mat4 new_shadow_view_proj = g_shadow_pass->Render(&csm_info, rfc->rl_->GetRenderPackets());

    gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    glViewport(0, 0, (GLsizei)Environment.drawableWidth, (GLsizei)Environment.drawableHeight);

    mat4 view_mat, proj_mat;
    if(render_from_shadow_camera)
    {
        view_mat = rfc->shadow_view_;
        proj_mat = new_shadow_view_proj * rfc->shadow_inv_view_;
    }
    else
    {
        view_mat = rfc->view_;
        proj_mat = rfc->proj_;
    }

    ShapeRenderer shape_renderer;
    shape_renderer.setup(view_mat, proj_mat);
    mat4 shadow_view_m = rfc->shadow_view_;
    vec4 lightdir;
    lightdir = -shadow_view_m.getRow(2);

    mat4 cascade_matrices[] = { csm_info.shadow_vp_[0], csm_info.shadow_vp_[1] };
    DWORD cascade_shadow_maps[] = { g_shadow_pass->GetShadowMap(0), g_shadow_pass->GetShadowMap(1) };
    HGOSTEXTURESAMPLER shadow_sampler = g_shadow_pass->GetSadowSampler();

    shape_renderer.set_shadow_params(
        lightdir, cascade_matrices, cascade_shadow_maps, shadow_sampler,
        csm_info.zfar_, res_man_load_texture("default"));

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();

    char rfc_info[128] = {0};
    sprintf(rfc_info, "rfc: %d rl: %d", rfc->frame_number_, rfc->rl_->GetId());
    rmt_BeginCPUSampleDynamic(rfc_info, 0);

//#define FORWARD_RENDERING
#if defined(FORWARD_RENDERING)
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
#if DO_BAD_THING_FOR_TEST
        // test: get transform on render thread which can't be done (because it is changing on game thread)
        // also cache miss on attempt to call function
        rp.m_ = rp.go_->GetTransform();
#endif
        if(rp.is_opaque_pass)
            shape_renderer.render(rp);
    }
    RenderParticles(rpl, view_mat, proj_mat);
    RenderDebugObjects(rpl, view_mat, proj_mat);

#else // !FORWARD_RENDERING

    g_deferred_renderer.RenderGeometry(rfc);
    g_deferred_renderer.RenderDirectionalLighting(rfc);
    g_deferred_renderer.RenderPointLighting(rfc);
    bool downsampled_particles = true;
    g_deferred_renderer.RenderForward(
        [&rpl, &view_mat, &proj_mat, downsampled_particles]() {
            if (!downsampled_particles)
                RenderParticles(rpl, view_mat, proj_mat);
            RenderDebugObjects(rpl, view_mat, proj_mat);
        });
    if (downsampled_particles) {
        g_deferred_renderer.RenderDownsampledForward(
            [&rpl, &view_mat, &proj_mat]() {
                RenderParticles(rpl, view_mat, proj_mat);
            },
            rfc->proj_);
    }

	if (g_is_in_editor) {
		g_obj_id_renderer.Render(rfc, g_deferred_renderer.GetSceneDepth());

		int xdelta, ydelta, wheeldelta;
		float xpos, ypos;
		DWORD buttonspressed;
		gos_GetMouseInfo(&xpos, &ypos, &xdelta, &ydelta, &wheeldelta, &buttonspressed);

		// TODO: this stalls GPU, make is async through FBO
		g_obj_under_cursor =
			g_obj_id_renderer.Readback((uint32_t)(Environment.drawableWidth * xpos),
									   (uint32_t)(Environment.drawableHeight * (1 - ypos)));
	}

    g_deferred_renderer.Present(Environment.drawableWidth,
                                Environment.drawableHeight);


#endif // FORWARD_RENDERING

    rmt_EndCPUSample();

    ReleaseRenderList(rfc->rl_);
    delete rfc;

    if(g_show_cascade_index>=0 && g_show_cascade_index<g_shadow_pass->GetNumCascades())
        render_fullscreen_quad(g_shadow_pass->GetShadowMap(g_show_cascade_index));
    
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




