#include "scene.h"
#include "particle_system.h"
#include "res_man.h"
#include "obj_model.h"
#include "rigid_body_object.h"
#include "renderer.h"
#include "render_utils.h"
#include "editor.h"

#include "utils/matrix.h"
#include "utils/camera.h"
#include "utils/math_utils.h"
#include "profiler/profiler.h"

#include <list>

#include "game/Level.h"

typedef std::vector<GameObject*> ObjList_t;
static ObjList_t g_world_objects;
static std::vector<GameObject*> g_init_pending_gos;
static std::vector<Component*> g_init_pending_comps;
static std::vector<GameObject*> g_destroy_pending_gos;
static std::vector<Component*> g_destroy_pending_comps;
// have separate arrays of concrete types?
static std::vector<std::vector<Component*>> g_components;
static std::vector<PointLight> g_light_list;


std::vector<IRenderable*> g_renderables_init_pending;
std::vector<IRenderable*> g_renderables_deinit_pending;
std::vector<IRenderable*> g_renderables;

static std::vector<std::pair<Component*, IRenderProxy*>> g_render_proxies;
static std::vector<std::pair<Component*, IRenderProxy*>> g_render_proxies_init_pending;
static std::vector<std::pair<Component*, std::pair<i32, IRenderProxy*>>> g_render_proxies_deinit_pending;

static uint32_t g_obj_id_under_cursor = scene::kInvalidObjectId;

static SceneViewInfo g_scene_view_info;
static StaticMesh* g_xy_quad = nullptr;

static ICameraController* g_cam_controller;

void scene_set_object_id_under_cursor(uint32_t obj_id) {
	g_obj_id_under_cursor = obj_id;
}
uint32_t scene_get_object_id_under_cursor() {
	return g_obj_id_under_cursor;
}

GameObject* scene_get_object_by_id(GameObjectId id) {
	if (id < scene::kFirstGameObjectId)
		return nullptr;

    for (auto go: g_world_objects) {
		if (go->GetId() == id)
			return go;
    }
	return nullptr;
}

const std::vector<PointLight>& scene_get_light_list() {
	return g_light_list;
}

void initialize_scene() {
    g_xy_quad = res_man_load_mesh2("xy_quad");

    g_components.resize((size_t)ComponentType::kCount);

    MeshObject* go = MeshObject::Create("floor");
    auto* tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 0, 0));
    tc->SetScale(vec3(50, 1, 50));
    scene_add_game_object(go);

    go = MeshObject::Create("sphere");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 0, 0));
    scene_add_game_object(go);

    go = MeshObject::Create("cube");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(1, 0, 0));
    scene_add_game_object(go);

    scene_add_game_object(Level::Create("level1"));

    go = MeshObject::Create("gizmo");
    tc = go->GetComponent<TransformComponent>();
    tc->SetScale(vec3(0.05f, 0.05f, 0.05f));
    scene_add_game_object(go);

    //FrustumObject *fo = FrustumObject::Create();
    //scene_add_game_object(fo);

    ParticleSystemObject *pso = ParticleSystemObject::Create();
    //pso->SetPosition(vec3(50, 0, 50));
    scene_add_game_object(pso);

    // create some point lights
    for (int i = 0; i < 10; ++i) {
        PointLight l;
        vec3 color = random_vec(0.f, 1.0f);
        float intensity = random(0.5f, 1.5f);
        l.color_ = vec4(color.x, color.y, color.z, intensity);
        l.radius_ = random(2.5f, 7.0f);
        vec3 p =
            random_vec(vec3(-50.0f, 5.0f, -50.0f), vec3(50.0f, 15.0f, 50.0f));
        l.pos = p;
        // default sphere mesh has radius 0.5f
        l.transform_ = translate(p) * mat4::scale(vec3(2*l.radius_));
        g_light_list.push_back(l);
    }
}

static void scene_update_pending(struct RenderFrameContext *rfc);
void finalize_scene() {

    res_man_release_mesh2(g_xy_quad);

    ObjList_t::const_iterator it = g_world_objects.begin();
    ObjList_t::const_iterator end = g_world_objects.end();
    for (; it != end; ++it) {
        GameObject *go = *it;
        go->SetState(GameObject::kPendingDestroy);
        //TOOD: should we just remove them from the scene?
        delete go;
    }
    RenderFrameContext nullctx;

    // a bit of a hack, because destruction also schedules
    // destroy render resources, we need to "tick" render
    // context and then again call scene_update_pending
    // to delete all dependent components. We cannot just
    // delete components without waiting for render commands_
    // to finish, because it is a same class at the moment
    // we will just crash
    scene_update_pending(&nullctx);
    for (auto& cmd : nullctx.commands_) {
        cmd();
    }
    // 2x because of 1 frame delay when deleting render proxies
    scene_update_rt_proxies(&nullctx);
    scene_update_rt_proxies(&nullctx);
    scene_update_pending(&nullctx);

    //debug:
    printf("proxies: %d init pending:%d deinit pending:%d\n", (int)g_render_proxies.size(),
        (int)g_render_proxies_init_pending.size(), (int)g_render_proxies_deinit_pending.size());
}

const SceneViewInfo& scene_get_view_info() {
    return g_scene_view_info;
}

void scene_update(const camera *cam, const bool b_update_simulation, const float dt) {

	if (!b_update_simulation)
        return;

    // fill per frame view info
    g_scene_view_info.view_mat_ = cam->get_view();
    g_scene_view_info.inv_view_mat_ = cam->get_inv_view();
    g_scene_view_info.proj_mat_ = cam->get_projection();
    g_scene_view_info.fov_ = cam->get_fov();
    g_scene_view_info.aspect_ = cam->get_aspect();

    ObjList_t::const_iterator it = g_world_objects.begin();
    ObjList_t::const_iterator end = g_world_objects.end();

    // TODO: this was called all the time, probably because transform components should be updated
    // in any case, keep an eye
    // update transform components
    {
    SCOPED_ZONE_N(UpdateComponents,0);
    for(int t=0;t<(int)ComponentType::kCount;++t) {
        for(auto comp: g_components[t]) {
            gosASSERT(comp->getState() == Component::kInitialized);
            comp->UpdateComponent(dt);
        }
    }
    }

    SCOPED_ZONE_N(go_Update,0);

    std::vector<GameObject*> pending_destroy;
    for (; it != end; ++it) {
        GameObject *go = *it;
        const GameObject::State state = go->GetState();
        if(state == GameObject::kInitialized) {
            go->Update(dt);
        } else if(state == GameObject::kPendingDestroy) {
            pending_destroy.push_back(go);
        }

		// if object is frustum object.... and we want to update it
        if (0) {
            camera loc_cam = *cam;
            loc_cam.set_projection(45.0f, Environment.drawableWidth,
                    Environment.drawableHeight, 4.0f, 20.0f);
            ((FrustumObject *)go)->UpdateFrustum(&loc_cam);
        }
    }

    for(auto go: pending_destroy) {
        scene_delete_game_object(go);
    }
}


void scene_add_component__(Component* comp) {
    comp->Initialize();
    if(auto ri = comp->getRenderableInterface()) {
        g_renderables_init_pending.push_back(ri);
    }
    if(IRenderProxy* proxy = comp->CreateRenderProxy()) {
        g_render_proxies_init_pending.push_back(std::make_pair(comp, proxy));
    }

    g_init_pending_comps.push_back(comp);
}

void scene_delete_component(Component* comp) {
    auto& cmp_list = g_components[(uint32_t)comp->GetType()];
    auto cb = std::begin(cmp_list);
    auto ce = std::end(cmp_list);
    cmp_list.erase(std::remove(cb, ce, comp), ce);

    comp->Deinitialize();

    GameObject* go = comp->getGameObjectHandle().go_handle_;
    if(go) {
        go->DetachComponent(comp);
    } else {
        log_warning("No game object when destructing component (type:%d)\n", comp->GetType());
    }

    // need to be careful that only after renderable is destroyed we actually
    // delet component (handled now through states)
    // TODO: just remove if from component interface and instead have a separate
    // Renderable objects not related to original component class?
    if(auto ri = comp->getRenderableInterface()) {
        g_renderables_deinit_pending.push_back(ri);
        auto fit = std::find(g_renderables.begin(), g_renderables.end(), ri);
        assert(fit != g_renderables.end());
        g_renderables.erase(fit);
    }

    g_destroy_pending_comps.push_back(comp);
}

void scene_add_game_object(GameObject* go) {
    g_init_pending_gos.push_back(go);
}

// TODO: what will happen if object is still in g_render_init_pending?
void scene_delete_game_object(GameObject* go) {
    auto b = g_world_objects.begin();
    auto e = g_world_objects.end();
	auto it = std::find(b, e, go);
    assert(it!=e);
    if(it!=e) {

        (*it)->SetState(GameObject::kDestroyed);
	    g_world_objects.erase(it);

        // smells
        while(go->GetComponents().size()) {
            scene_delete_component(go->GetComponents()[0]);
        }
        g_destroy_pending_gos.push_back(go);
	}
}

void scene_attach_component(GameObject* go, Component* comp)
{
    // TODO: do we need to have this in GameObject? maybe just store correspondence in scene itself
    go->AttachComponent(comp);
}

void scene_detach_component(GameObject* go, Component* comp)
{
    // TODO: do we need to have this in GameObject? maybe just store correspondence in scene itself
    go->DetachComponent(comp);
}

// TODO: should this be in an editor.cpp? or even create a separate 3dview.cpp
void scene_draw_object_list() {
#if WITH_EDITOR
    ImGui::Begin("ObjList");
    int sel_index = -1;
    GameObject* sel_go = editor_get_selected_obj();
    int idx = 0;
    for(auto& go : g_world_objects) {
        if(go == sel_go) {
            sel_index = idx;
            break;
        }
        idx++;
    }
    DrawPropertySheetCollapsibleList("Objects", g_world_objects, 
            [](const auto& item, size_t index) {
                return item->GetName();
            },
            &sel_index);

    if(sel_index !=-1) {
        editor_set_selected_obj(g_world_objects[sel_index]);
    }
    
    ImGui::End();
#endif
}

void scene_render_update(struct RenderFrameContext *rfc, bool is_in_editor_mode, bool b_exclusive_3dview) {

    RenderList *frame_render_list = rfc->rl_;

    if (frame_render_list->GetCapacity() < g_world_objects.size()) {
        frame_render_list->ReservePackets(g_world_objects.size());
    }

	for (const GameObject* go : g_world_objects) {
		const auto* tc = go->GetTransformInterface();
		const int icon_id = go->GetIconID();

		if (is_in_editor_mode && tc && icon_id > 0) {
			RenderMesh mesh = tmp_rm(g_xy_quad);
			vec3 pos = tc->Transform(vec3(0));
			add_debug_mesh_constant_size_px(rfc, &mesh, 1, vec4(0, 0.5, 1, 1),
											mat4::translation(pos), 20, go->GetId());

			if (true) {
				// just a test that quaternions do same thing as matrix, should be in a
				// unit test
				vec3 loc = vec3(1, 1, 1);
				vec3 pos1 = tc->Transform(loc);
				vec4 pos2 = tc->GetTransform() * vec4(loc, 1);
                if(lengthSqr(pos1 - pos2.xyz()) >= 0.0001f) {
                    printf("error\n");
                }
				assert(lengthSqr(pos1 - pos2.xyz()) < 0.0001f);
			}
		}

        go->AddRenderPackets(rfc);
	}

	rfc->point_lights_ = g_light_list;

    { SCOPED_ZONE_N(RenderUpdateComponents,0);
        for(int t=0;t<(int)ComponentType::kCount;++t) {
            for(auto comp: g_components[t]) {
                gosASSERT(comp->getState() == Component::kInitialized);
                comp->RenderUpdateComponent(rfc);
            }
        }
    }

    for(IRenderable* ri: g_renderables) {
        if(ri->IsRenderInitialized()) {
            ri->AddRenderPackets(rfc);
        }
    }

    if(!b_exclusive_3dview) {
        scene_draw_object_list();
    }

    scene_update_pending(rfc);
}

void scene_update_pending(struct RenderFrameContext *rfc) {
    // NOTE: this might not be true anymore
	// update objects pending creation or destruction after update loop because if we add
	// object to world we first want it to update its components in scene_update
	// so by doing it at the end of this function objects wich are added here in
	// g_components will be updated next frame and ony drawn after update

	// schedule render resource (de)initialization for pending objects
	for (IRenderable* ri: g_renderables_init_pending) {
        ri->StartInit(rfc);
        assert(std::find(g_renderables.begin(), g_renderables.end(), ri) == g_renderables.end());
        g_renderables.push_back(ri);
	}
    g_renderables_init_pending.clear();

	for (IRenderable* ri: g_renderables_deinit_pending) {
        ri->StartDeinit(rfc);
	}
    g_renderables_deinit_pending.clear();

	// update init pending components array 
    for (size_t i = 0; i < g_init_pending_comps.size(); ++i) {
        Component* c = g_init_pending_comps[i];
        if(c->getState() == Component::kInitialized) {
            g_components[(uint32_t)c->GetType()].push_back(c);
            g_init_pending_comps[i] = nullptr;
        }
    }
	{
		auto b = std::begin(g_init_pending_comps);
		auto e = std::end(g_init_pending_comps);
		g_init_pending_comps.erase(
			std::remove_if(b, e, [](const auto& p) { return p == nullptr; }), e);
	}

    for (size_t i = 0; i < g_init_pending_gos.size(); ++i) {
        GameObject* go = g_init_pending_gos[i];
        bool all_initialized = true;
        for(auto c: go->GetComponents()) {
            all_initialized &= (c->getState() == Component::kInitialized);
        }
        // unconditional addition, do not wait for all components to be initialized
        // as in this case we may e.g. draw mesh before calling Update() first
        if(true || all_initialized) {
            g_init_pending_gos[i] = nullptr;
			assert(std::find(g_world_objects.begin(), g_world_objects.end(), go) ==
				   g_world_objects.end());
			g_world_objects.push_back(go);
        }
    }
	{
		auto b = std::begin(g_init_pending_gos);
		auto e = std::end(g_init_pending_gos);
		g_init_pending_gos.erase(
			std::remove_if(b, e, [](const auto& p) { return p == nullptr; }), e);
	}

    // destroy game objects
    for (GameObject* go: g_destroy_pending_gos) {
        assert(std::find(g_world_objects.begin(), g_world_objects.end(), go) ==
                g_world_objects.end());
        delete go;
    }
    g_destroy_pending_gos.clear();

    // destroy components
    for (size_t i = 0; i < g_destroy_pending_comps.size(); ++i) {
        Component* c = g_destroy_pending_comps[i];
        if(c->getState() == Component::kUninitialized) {
            g_destroy_pending_comps[i] = nullptr;

            if(IRenderProxy* proxy = c->GetRenderProxy()) {
                g_render_proxies_deinit_pending.push_back(std::make_pair(c, std::make_pair(1, proxy)));

                //TODO: should be O(1)
                {
                    const size_t old_size = g_render_proxies.size();
                    auto b = std::begin(g_render_proxies);
                    auto e = std::end(g_render_proxies);
                    g_render_proxies.erase(
                            std::remove_if(b, e, [c](const auto& p) { return p.first == c; }), e);
                    gosASSERT(g_render_proxies.size() == old_size - 1);
                }
            }

            delete c;
        }
    }
	{
		auto b = std::begin(g_destroy_pending_comps);
		auto e = std::end(g_destroy_pending_comps);
		g_destroy_pending_comps.erase(
			std::remove_if(b, e, [](const auto& p) { return p == nullptr; }), e);
	}

}

void scene_update_rt_proxies(struct RenderFrameContext* rfc) {

    for(auto p: g_render_proxies_init_pending) {
        p.second->Initialize(rfc);
        g_render_proxies.push_back(p);
    }
    g_render_proxies_init_pending.clear();

    for(auto p: g_render_proxies) {
        p.second->AddRenderPackets(rfc);
    }

    for(auto& p: g_render_proxies_deinit_pending) {
        // NOTE: p.first - is a component pointer and is most probably
        // already destructed, use just for debugging
        // NOTE: could use same appoah as in res_man, so that component
        // will not be destroyed until barrier will be set from rt

        // delay destroy one frame because render proxy could be scheduled in update before owning 
        // component was destroyed (could happen if it is destroyed by another componen/objcect 
        // later in update loop
        if(0 == p.second.first--) {
            delete p.second.second;
        } else {
            p.second.second->Deinitialize(rfc);
        }
    }

	{
		auto b = std::begin(g_render_proxies_deinit_pending);
		auto e = std::end(g_render_proxies_deinit_pending);
		g_render_proxies_deinit_pending.erase(
			std::remove_if(b, e, [](const auto& p) { return p.second.first==-1; }), e);
	}
}

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3 &ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj) {

    using el_t = std::pair<float, GameObject *>;
    for (auto &obj : g_world_objects) {

        const auto* tc = obj->GetComponent<TransformComponent>();

        vec3 os_orig;
        vec3 os_dir;

        if (tc) {
            // transform to object space
            float inv_w[16];
            glu_InvertMatrixf(tc->GetTransform(), inv_w);
            const mat4 inv_world =
                mat4(inv_w[0], inv_w[1], inv_w[2], inv_w[3], inv_w[4], inv_w[5],
                     inv_w[6], inv_w[7], inv_w[8], inv_w[9], inv_w[10],
                     inv_w[11], inv_w[12], inv_w[13], inv_w[14], inv_w[15]);

            os_orig = (inv_world * vec4(ws_orig, 1)).xyz();
            os_dir = normalize((inv_world * vec4(ws_dir, 0)).xyz());
        } else {
            os_orig = ws_orig;
            os_dir = normalize(ws_dir);
        }

        // TODO: all mesh components should add their meshes to the scene!
        MeshComponent* mc = obj->GetComponent<MeshComponent>();
        vec3 t = intersect_aabb_ray(mc->GetAABB(), os_orig, os_dir);
        if (t.z && t.x >= 0.0f) {
            // transform t back to world space
            const vec3 int_pos = os_orig + os_dir * t.x;
            const vec3 int_wpos =
                tc ? (tc->GetTransform() * vec4(int_pos, 1.0f)).xyz() : int_pos;
            const float dist = length(int_wpos - ws_orig);
            out_obj.push_back(std::make_pair(dist, obj));
        }
    }
}

void scene_set_camera_controller(class ICameraController* cc) {
    g_cam_controller = cc;
}

ICameraController* scene_get_camera_controller() {
    return g_cam_controller;
}

