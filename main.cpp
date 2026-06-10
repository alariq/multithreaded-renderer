#include "engine/utils/timing.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/camera.h"
#include "engine/utils/obj_loader.h"
#include "engine/utils/matrix.h"
#include "engine/utils/ringbuffer.h"
#include "engine/utils/logging.h"
#include "engine/utils/imgui_property_list.h"
#include "engine/profiler/profiler.h"
#include "engine/gameos.hpp"

#include "pbd/pbd.h"
#include "pbd_test.h"
#include "scene.h"
#include "editor.h"
#include "res_man.h"
#include "renderer.h"
#include "shadow_renderer.h"
#include "particle_system.h"
#include "debug_renderer.h"
#include "text2d_renderer.h"
#include "deferred_renderer.h"
#include "forward_renderer.h"
#include "obj_id_renderer.h"

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

struct RetiredRTContext {
    u32 id_under_cursor;
    int frame_idx_dbg;
};
SPSCRingBufferT<RetiredRTContext, 3> gRetiredCtxs;

bool g_is_in_editor = WITH_EDITOR ? true : false;
bool g_render_initialized_hack = false;
bool g_update_simulation = false;
bool g_update_simulation_step_by_step = false;
bool g_exclusive_3dview = WITH_EDITOR ? false : true; // 3dview occupies whole window

DWORD g_htexture = 0;
ShadowRenderPass* g_shadow_pass = nullptr;
uint32_t g_show_cascade_index = 2;
bool render_from_shadow_camera = false;

DeferredRenderer g_deferred_renderer;
ForwardRenderer g_forward_renderer;
ObjIdRenderer g_obj_id_renderer;

camera g_shadow_camera;

void __stdcall Init(void)
{
    printf("::Init\n");

    extern void tests_run();
    tests_run();

    vec3 light_dir = normalize(vec3(1.0,-1.0,1.0));

    vec3 up = vec3(0,1,0);

    vec3 right = normalize(cross(up, light_dir));
    up = normalize(cross(light_dir, right));

    mat4 shadow_view = mat4::identity();
    // negative light dir by convention
    camera::compose_view_matrix(&shadow_view, right, up, -light_dir,
                                -light_dir * 50.0f);

    g_shadow_camera.set_ortho_projection(-30, 30, 30, -30, 0.1f, 200.0f);
    g_shadow_camera.set_view(shadow_view);

    unified_pbd_init();
    pbd_create_simulation();
    pbd_create_collision_detection();

    initialize_res_man();
}

void __stdcall Deinit(void)
{
    g_obj_id_renderer.Deinit();

    DeleteRenderLists();

    g_deferred_renderer.Deinit();

    delete g_shadow_pass;

    ParticleSystemManager::Instance().DestroyRenderResources();

    finalize_editor();
    finalize_scene();
    finalize_res_man();

    pbd_destroy_collision_detection();
    pbd_destroy_simulation();
    unified_pbd_deinit();

    printf("::Deinit\n");
}

void __stdcall Update(void)
{
    RenderFrameContext* rfc = new RenderFrameContext();
	rfc->commands_.clear();

    RetiredRTContext rrtc;
    if(gRetiredCtxs.pop(rrtc)) {
        scene_set_object_id_under_cursor(rrtc.id_under_cursor);
    }

    static bool initialization_done = false;
    if(!initialization_done)
    {
        gos_SetRelativeMouseMode(!g_is_in_editor);
        initialize_scene();
        initialize_editor();
        initialization_done = true;
    }

    static uint64_t start_tick = timing::gettickcount();

    const uint64_t end_tick = timing::gettickcount();
    const uint64_t delta_tick = end_tick - start_tick;
    float dt_sec =(float)((double)timing::ticks2ns(delta_tick)/1e9);
    // stop-on-breakpoint-proof dt
    dt_sec = clamp(dt_sec, 0.0f, 0.033f*10);

    //printf("dt: %zd\n", dt);

    start_tick = timing::gettickcount();

    ICameraController* game_cc = scene_get_camera_controller();
    ICameraController* editor_cc = editor_get_cam_controller();
    ICameraController* main_cc = g_is_in_editor || !game_cc ? editor_cc : game_cc;
    camera main_cam = main_cc->GetCamera();

    if(WITH_EDITOR) {
        if(gos_GetKeyStatus(KEY_F) == KEY_PRESSED && gos_GetKeyStatus(KEY_LCONTROL)) {
            g_exclusive_3dview = !g_exclusive_3dview;
        }

        if(editor_get_3dview_hovered()) { // TODO: move into separate update_3dvew() in editor.cpp ?

            if(gos_GetKeyStatus(KEY_SPACE) == KEY_PRESSED)
                g_update_simulation = !g_update_simulation;

            if(gos_GetKeyStatus(KEY_F1) == KEY_PRESSED)
                g_update_simulation_step_by_step = !g_update_simulation_step_by_step;

            if(gos_GetKeyStatus(KEY_TAB) == KEY_PRESSED)
            {
                if(!g_is_in_editor && game_cc) {
                    float xrot = camera::getXrot(game_cc->GetCamera().get_view());
                    editor_cam_controller_set_transform(game_cc->GetCamera().get_pos(), xrot);
                }
                g_is_in_editor = !g_is_in_editor;
                gos_SetRelativeMouseMode(!g_is_in_editor);
            }
        }

        render_from_shadow_camera = gos_GetKeyStatus(KEY_F3) ? true : false;
        g_show_cascade_index += gos_GetKeyStatus(KEY_LCONTROL) && gos_GetKeyStatus(KEY_K) == KEY_PRESSED ? 1 : 0;
        if(g_show_cascade_index>2)
            g_show_cascade_index = 0;
    }

    // NOTE: we get 3dview hover/focused states here before they are updated in editor_calc_3dview()
    // to help mouse cursor wraparound otherwise when mouse is moving fast and goes out of the window 
    // it is not hovered anymore and then UpdateCamera early outs, need to check if 
    // "not hovered and mouse movement did not start inside window"
    const bool b_3dview_hovered = editor_get_3dview_hovered();

    // TODO: we call it even in game mode becase game can run in editor viewport
    // need to add something like WITH_EDITOR, so we can distinguish between
    // "editor mode" and "is editor compiled"
    ivec4 scene_viewport = editor_calc_3dview(g_exclusive_3dview, g_deferred_renderer.GetSceneColour());

    // TODO: move to array of systems ?
    if(g_update_simulation) {
        {
            SCOPED_ZONE_N(pbd_simulate, 0);
            pbd_simulate(dt_sec);
        }
        {
            SCOPED_ZONE_N(unified_pbd_timestep, 0);
            unified_pbd_update(dt_sec);
        }
        {
            SCOPED_ZONE_N(ParticleSystemManager_Update, 0);
            ParticleSystemManager::Instance().Update(dt_sec);
        }
    }


    res_man_update();


    scene_update(&main_cam, g_update_simulation, dt_sec);

    if (g_update_simulation) {
        if (g_update_simulation_step_by_step) {
            g_update_simulation = false;
        }
    }

    if(g_is_in_editor) {
	    editor_update(&main_cam, dt_sec);
    }

    if(b_3dview_hovered)
        editor_cc->Update(dt_sec);
    if(game_cc) game_cc->Update(dt_sec);

    main_cam = main_cc->GetCamera();

	// prepare list of objects to render
    BEGIN_ZONE_N(acq_zone, AcquireRenderList, 0);
    RenderList* frame_render_list = AcquireRenderList();
    END_ZONE(acq_zone);

    // setup render frame context
    CSMInfo csm_info;
    fill_csm_frustums(&csm_info, &main_cam, &g_shadow_camera);
    rfc->frame_number_ = GetCurrentFrame();
    rfc->rl_ = frame_render_list;
    rfc->csm_info_ = csm_info;
	rfc->inv_view_ = main_cam.get_inv_view();
    rfc->view_ = main_cam.get_view();
    rfc->proj_ = main_cam.get_projection();
    rfc->inv_proj_ = main_cam.get_inv_projection();
    rfc->z_near_ = main_cam.get_near();
    rfc->z_far_ = main_cam.get_far();
    rfc->fov_ = main_cam.get_fov();
    rfc->aspect_ = main_cam.get_aspect();
    rfc->b_is_perspective_ = main_cam.get_is_perspective();
    g_shadow_camera.get_view(&rfc->shadow_view_);
    rfc->shadow_inv_view_ = g_shadow_camera.get_inv_view();
    // TODO: using viewport here might not work if we render to some 
    // pass which uses different WxH, clients need to use per pass current viewport
    rfc->viewport_ = scene_viewport;
    //

    char listidx_str[32] = {0};
    sprintf(listidx_str, "list_idx: %d", frame_render_list->GetId());
    BEGIN_ZONE_DYNAMIC_N(list_idx, listidx_str, 0);

    if(g_render_initialized_hack)
    {
        {
            SCOPED_ZONE_N(scene_render_update, 0);
            scene_render_update(rfc, g_is_in_editor, g_exclusive_3dview);
        }

        //if(compiled with editor)
		{
            SCOPED_ZONE_N(editor_render_update, 0);
			editor_render_update(rfc, g_is_in_editor, g_exclusive_3dview);
#if WITH_EDITOR
            if(!g_exclusive_3dview) {
                ImGui::Begin("Property");
                //imgui_props::DrawPropertySheetTable("camera", *main_cc);
                main_cc->DrawPolymorphicPropertySheet("camera controller");
                ImGui::End();
            }
#endif
        }
        {
            SCOPED_ZONE_N(ParticleSystemManager_RenderUpdate, 0);
            // add particle systems tasks to render list
            ParticleSystemManager::Instance().Render(rfc);
        }
	}

    res_man_schedule_rt_requests(rfc);

    SetRenderFrameContext(rfc);

    END_ZONE(list_idx);
}

class ShapeRenderer {

	mat4 view_;
	mat4 proj_;
    const DWORD* shadow_maps_;
    const mat4* shadow_matrices_;
    vec4 lightdir_;
    const float* zfar_;
    HGOSTEXTURESAMPLER shadow_sampler_;
    TextureHandle def_tex_;

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
                           const TextureHandle def_tex) {
        shadow_matrices_ = shadow_matrices;
        shadow_maps_ = shadow_maps;
        shadow_sampler_ = shadow_sampler;
        lightdir_ = lightdir;
        zfar_ = zfar;
        def_tex_ = def_tex;
    }

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");

        gos_SetRenderState(gos_State_Texture, ro.tex_id_ ? ro.tex_id_ : texture_res_get_gpu_id(def_tex_));
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
    
    gos_SetRenderState(gos_State_ZCompare, 0);
    gos_SetRenderState(gos_State_ZWrite, 1);
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

    StaticMesh* fs_quad = res_man_load_mesh2("fs_quad");
    if(fs_quad->rd.vb_)
        gos_RenderIndexedArray(fs_quad->rd.ib_, fs_quad->rd.vb_, fs_quad->rd.vdecl_, fs_quad->prim_type_);

}

void render_fullscreen_quad(uint32_t tex_id)
{
    render_quad(tex_id, vec4(1,1,0,0), nullptr);
}

void __stdcall Render(void)
{
    SCOPED_ZONE_NAMED(RenderZ, 0);
    static bool initialized = false;
    // should this be a command added by Update to render thread?
    if(!initialized)
    {

        g_shadow_pass = new ShadowRenderPass();
        if(!g_shadow_pass->Init(1024, 2))
        {
            SPEW(("SHADOWS", "Failed to init shadow pass"));
        }

        gos_AddRenderMaterial("directional_shadow");
        gos_AddRenderMaterial("particle");

        initialize_render_editor();

		uint32_t w = (uint32_t)Environment.drawableWidth;
		uint32_t h = (uint32_t)Environment.drawableHeight;
        g_deferred_renderer.Init(w, h);
        g_obj_id_renderer.Init(w, h);
        g_forward_renderer.Init();

        initialized = true;

        g_render_initialized_hack = true;
    }

    ParticleSystemManager::Instance().InitRenderResources();

    RenderFrameContext* rfc_mut = (RenderFrameContext*)GetRenderFrameContext();
    const RenderFrameContext* rfc = rfc_mut;
    assert(rfc && rfc->frame_number_ == RendererGetCurrentFrame());

	// process all scheduled commands
	{
		SCOPED_ZONE_N(Commands, 0);
		for (auto& cmd : rfc->commands_) {
			cmd();
		}
	}

    // update render proxies
    scene_update_rt_proxies(rfc_mut);

    // TODO: confine mouse pointer in 3d view if in game mode?

    const uint32_t view_w = rfc->viewport_.z;
    const uint32_t view_h = rfc->viewport_.w;
    gos_SetRenderViewport(0, 0, view_w, view_h);

	{
		SCOPED_ZONE_N(DebugDraw, 0);
		for (auto& dp : rfc->rl_->GetDebugPrimitives()) {
			switch (dp.type_) {
			case DebugPrimitive::kLine:
                if(dp.line_.vts)
                    gos_AddLines(dp.line_.vts, dp.line_.colours, dp.colour_, dp.count_, &dp.transform_);
                else
                    gos_AddLine(dp.line_.s, dp.line_.e, dp.colour_, &dp.transform_);
				break;
			case DebugPrimitive::kPoint:
				gos_AddPoints(dp.point_.vts, dp.count_, dp.colour_, dp.point_.size,
							  &dp.transform_);
				//delete[] dp.point_.vts;
				break;
			case DebugPrimitive::kQuad:
				gos_AddQuad(dp.quad_.size, dp.colour_, dp.quad_.tex_id, &dp.transform_,
							dp.b_two_sided_);
				break;
			}
		}
	}


    const CSMInfo& csm_info = rfc->csm_info_;

    // render shadows first
    mat4 new_shadow_view_proj = g_shadow_pass->Render(&csm_info, rfc->rl_->GetRenderPackets());


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
        csm_info.zfar_, res_man_load_texture2("default"));

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    const TextRenderPacketList_t& trpl = rfc->rl_->GetTextPackets();

    char rfc_info[128] = {0};
    sprintf(rfc_info, "rfc: %d rl: %d", rfc->frame_number_, rfc->rl_->GetId());
    BEGIN_ZONE_DYNAMIC_N(rfc_zone, rfc_info, 0);

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

    if (downsampled_particles) {
        g_deferred_renderer.RenderDownsampledForward(
            [&rpl, &view_mat, &proj_mat]() {
                RenderParticles(rpl, view_mat, proj_mat);
            },
            rfc->proj_);
    }

    g_deferred_renderer.RenderForward(
        [&rpl, &trpl, &view_mat, &proj_mat, rfc, downsampled_particles]() {
            if (!downsampled_particles)
                RenderParticles(rpl, view_mat, proj_mat);
            g_forward_renderer.Render(rfc);
            RenderDebugObjects(rpl, view_mat, proj_mat);
            gos_RenderDebugPrimitives(view_mat, proj_mat);
            RenderText2D(trpl, view_mat, proj_mat);

        });
	if (g_is_in_editor && !(gos_GetKeyStatus(KEY_LMOUSE) == KEY_HELD)) {
		g_obj_id_renderer.Render(rfc, g_deferred_renderer.GetSceneDepth());

		int xdelta, ydelta, wheeldelta;
		float xpos, ypos;
		DWORD buttonspressed;
		gos_GetMouseInfo(&xpos, &ypos, &xdelta, &ydelta, &wheeldelta, &buttonspressed);

        // mouse to texture space
        uint32_t tx = Environment.drawableWidth*xpos - (uint32_t)rfc->viewport_.x;
        uint32_t ty = (uint32_t)rfc->viewport_.w - (Environment.drawableHeight*ypos - (uint32_t)rfc->viewport_.y);
		u32 obj_id = g_obj_id_renderer.Readback(tx, ty);

        if(!gRetiredCtxs.push({.id_under_cursor = obj_id, .frame_idx_dbg = rfc->frame_number_})) {
            log_warning("not enough space for retired context");
        }
	}

    if(g_exclusive_3dview) {
        g_deferred_renderer.Present(view_w, view_h);
    }


#endif // FORWARD_RENDERING

    END_ZONE(rfc_zone);

    ReleaseRenderList(rfc->rl_);
    delete rfc;

    if(g_show_cascade_index>=0 && g_show_cascade_index<g_shadow_pass->GetNumCascades())
        render_fullscreen_quad(g_shadow_pass->GetShadowMap(g_show_cascade_index));
    
}

void GetGameOSEnvironment(const char* cmdline)
{
    (void)cmdline;
    Environment.displayIndex = 1;
    Environment.screenWidth = -1;
    Environment.screenHeight = -1;
    Environment.bitDepth = -1;

    Environment.InitializeGameEngine = Init;
    Environment.DoGameLogic = Update;
    Environment.TerminateGameEngine = Deinit;
    Environment.UpdateRenderers = Render;
}

