#include "scene.h"
#include "particle_system.h"
#include "res_man.h"
#include "obj_model.h"
#include "rigid_body_object.h"
#include "renderer.h"
#include "render_utils.h"

#include "utils/matrix.h"
#include "utils/camera.h"
#include "utils/math_utils.h"
#include "utils/timing.h"
#include <mutex>
#include <list>

typedef std::list<GameObject*> ObjList_t;
static ObjList_t g_world_objects;
static std::vector<GameObject*> g_render_init_pending;
static std::vector<GameObject*> g_render_destroy_pending;
// have separate arrays of concrete types?
static std::vector<std::vector<Component*>> g_components;
static std::vector<PointLight> g_light_list;

static uint32_t g_obj_id_under_cursor = scene::kInvalidObjectId;

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

void initialize_scene(const struct camera *cam, struct RenderFrameContext *rfc) {
    (void)cam;
    g_components.resize((size_t)ComponentType::kCount);

    const uint32_t NUM_OBJECTS = 3;
    for (uint32_t i = 0; i < NUM_OBJECTS; ++i) {
        MeshObject *go = MeshObject::Create("N");
        vec3 base_pos = random_vec(vec3(-15, 5, -15), vec3(15, 10, 15));
        auto* tr_comp = go->GetComponent<TransformComponent>();
        tr_comp->SetPosition(base_pos);
        tr_comp->SetScale(random_vec(vec3(1), vec3(2.5)));
        vec3 ang = random_vec(vec3(0), vec3(2.0f * 3.1415f));
        quaternion rot_q = euler_to_quat(ang.x, ang.y, ang.z);
        tr_comp->SetRotation(rot_q);
#if 1
        float start_time =
            (float)timing::ticks2ms(timing::gettickcount()) / 1000;
        const float amplitude = random(12.0f, 30.0f);
        const float phase = random(0.0f, 2.0f * 3.1415f);
        go->SetUpdater([base_pos, start_time, amplitude,
                        phase](float dt, MeshObject *gobj) mutable {
            auto* tc = gobj->GetComponent<TransformComponent>();
            vec3 p = tc->GetPosition();
            p.y =
                base_pos.y + amplitude * 0.5f * (sin(phase + start_time) + 1.0f);
            start_time += dt;
#if DO_BAD_THING_FOR_TEST
            go->SetPosition(random_vec(vec3(-10), vec3(10)) +
                            vec3(1000, 1000, 1000));
            // timing::sleep(100000);
#endif
            tc->SetPosition(p);
        });
#endif

        scene_add_game_object(go);
    }

    MeshObject* go = MeshObject::Create("column");
    auto* tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 0, 0));
    tc->SetScale(vec3(4, 8, 4));
    scene_add_game_object(go);

    go = MeshObject::Create("floor");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 0, 0));
    tc->SetScale(vec3(50, 1, 50));
    scene_add_game_object(go);

    go = MeshObject::Create("axes");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 20, 0));
    tc->SetScale(vec3(10, 10, 10));
    scene_add_game_object(go);

    go = MeshObject::Create("torus");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 30, 0));
    tc->SetScale(vec3(10, 10, 10));
    scene_add_game_object(go);

    camera loc_cam = *cam;
    // drawableWidth/Height may not be filled yet as renderer might not be initialized yet
    loc_cam.set_projection(45, Environment.screenWidth,
                           Environment.screenHeight, 2.0f, 20.0f);
    FrustumObject *fo = FrustumObject::Create(&loc_cam);
    scene_add_game_object(fo);

    ParticleSystemObject *pso = ParticleSystemObject::Create();
    scene_add_game_object(pso);

	RigidBodyObject *rb_floor = RigidBodyObject::Create(vec3(4.5f, 1, 4.5f));
    rb_floor->setKinematic(true);
    rb_floor->SetTransform(vec3(0, 0, 0), quaternion(vec3(1,0,0), M_PI/6.0f));
	scene_add_game_object(rb_floor);

	RigidBodyObject *rb_floor2 = RigidBodyObject::Create(vec3(5.5f, 2, 5.5f));
    rb_floor2->setKinematic(true);
    rb_floor2->SetTransform(vec3(0.0f, -0.0f, 4.5f), quaternion(vec3(1,0,0), -M_PI/6.0f));
	scene_add_game_object(rb_floor2);

    vec3 er = random_vec(vec3(0), vec3(2.0f * 3.1415f));
    quaternion qrot = euler_to_quat(er.x, er.y, er.z);
	RigidBodyObject *rb_cube = RigidBodyObject::Create(vec3(1, 1, 1));
    rb_cube->SetTransform(vec3(0, 3, 0), qrot);
	scene_add_game_object(rb_cube);

	RigidBodyObject *rb_floor3 = RigidBodyObject::Create(vec3(25.5f, 0.5f, 25.5f));
    rb_floor3->setKinematic(true);
    rb_floor3->SetTransform(vec3(0.0f, -2.0f, 4.5f), quaternion::identity());
	scene_add_game_object(rb_floor3);

    // make vilage
    const float rot[] = {0, 150, 30, 90, 55};
    const float scales[] = {0.1f, 0.07f, 0.12f, 0.08f, 0.1f};
    const vec2 pos[] = {vec2(10, 10), vec2(-10, -10), vec2(30, 35),
                        vec2(10, -25), vec2(-30, 5)};
    for (int i = 0; i < 5; ++i) {
        go = MeshObject::Create("single_room_building");
        auto* t = go->GetComponent<TransformComponent>();
        vec3 ang = vec3(0.0f, -rot[i] * 3.1415f / 180.0f, 0.0f);
        t->SetRotation(euler_to_quat(ang.x, ang.y, ang.z));
        t->SetPosition(vec3(pos[i].x, 0.0f, pos[i].y));
        t->SetScale(vec3(scales[i]));
        scene_add_game_object(go);
    }

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
        l.transform_ = translate(p) * mat4::scale(vec3(l.radius_));
        g_light_list.push_back(l);
    }
}

void finalize_scene() {

    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();
    for (; it != end; ++it) {
        GameObject *go = *it;
        delete go;
    }
}

void scene_update(const camera *cam, const bool b_update_simulation, const float dt) {
    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();

    // update transform components
    for(int t=0;t<(int)ComponentType::kCount;++t) {
        for(auto comp: g_components[t]) {
            comp->UpdateComponent(dt);
		}
	}

	if (b_update_simulation) {


		for (; it != end; ++it) {
			GameObject *go = *it;
			go->Update(dt);

			// if object is frustum object.... and we wnt to update it
			if (0) {
				camera loc_cam = *cam;
				loc_cam.set_projection(45.0f, Environment.drawableWidth,
									   Environment.drawableHeight, 4.0f, 20.0f);
				((FrustumObject *)go)->UpdateFrustum(&loc_cam);
			}
		}
	}
}

void scene_add_game_object(GameObject* go) {
    g_render_init_pending.push_back(go);
}

// TODO: what will happen if object is still in g_render_init_pending?
void scene_delete_game_object(GameObject* go) {
    auto b = g_world_objects.begin();
    auto e = g_world_objects.end();
	auto it = std::find(b, e, go);
    assert(it!=e);
    if(it!=e) {

	    g_world_objects.erase(it);

		for (Component *comp : go->GetComponents()) {
            auto& cmp_list = g_components[(uint32_t)comp->GetType()];
			auto cb = std::begin(cmp_list);
			auto ce = std::end(cmp_list);
            cmp_list.erase(std::remove(cb, ce, comp), ce);
		}

        g_render_destroy_pending.push_back(go);
	}
}

void scene_render_update(struct RenderFrameContext *rfc, bool is_in_editor_mode) {

    RenderList *frame_render_list = rfc->rl_;

    if (frame_render_list->GetCapacity() < g_world_objects.size())
        frame_render_list->ReservePackets(g_world_objects.size());

    // TODO: what to do about destroy
    for(Component* comp: g_components[(int)ComponentType::kMesh]) {
        MeshComponent* mesh_comp = (MeshComponent*)comp;
        if(!mesh_comp->IsRenderInitialized()) {
	        ScheduleRenderCommand(rfc, [mesh_comp]() { mesh_comp->DoInitRenderResources(); });
        } else {
            mesh_comp->AddRenderPackets(rfc);
        }
	}


	for (const GameObject *go : g_world_objects) {

		// TODO: check if visible
        // ...

        const auto* tc = go->GetComponent<TransformComponent>();
        // maybe instead create separate render thread render objects or make
        // all render object always live on render thread
        if (go->GetMesh())
        {
            RenderPacket *rp = frame_render_list->AddPacket();

            rp->mesh_ = *go->GetMesh();
            rp->m_ = tc ? tc->GetTransform() : mat4::identity();
			rp->id_ = go->GetId();
            rp->is_opaque_pass = 1;
            rp->is_render_to_shadow = 1;
            rp->is_transparent_pass = 0;
            rp->is_debug_pass = 0;
            rp->is_selection_pass = go->IsSelectable() && is_in_editor_mode;
#if DO_BAD_THING_FOR_TEST
            rp->go_ = go;
#endif
        }

        go->AddRenderPackets(rfc);

        if(is_in_editor_mode)
        {
            int icon_id = go->GetIconID();
            if(icon_id > 0 && tc) {
                RenderMesh* mesh = res_man_load_mesh("xy_quad");
				if (true) {
					// just a test that quaternions do same thing as matrix, should be in a unit test
					vec3 loc = vec3(1, 1, 1);
					vec3 pos = tc->Transform(loc);
					vec4 pos2 = tc->GetTransform() * vec4(loc, 1);
					assert(lengthSqr(pos - pos2.xyz()) < 0.0001f);
				}
				vec3 pos = tc->Transform(vec3(0));
                add_debug_mesh_constant_size_px(rfc, mesh, vec4(0,0.5,1,1), mat4::translation(pos), 10, go->GetId());
            }
        }
	}

	rfc->point_lights_ = g_light_list;

	// update objects pending creation or destruction after update loop because if we add
	// object to world we first want it to update its components in scene_update
	// so by doing it at the end of this fuction objects wich are added here in
	// g_components will be updated next frame and ony drawn after update

	// update render init pending array
    for (GameObject*& gameobj: g_render_init_pending) {
        if(gameobj->IsRenderInitialized()) {
			assert(std::find(g_world_objects.begin(), g_world_objects.end(), gameobj) ==
				   g_world_objects.end());
			g_world_objects.push_back(gameobj);
            for(Component* comp: gameobj->GetComponents()) {
                g_components[(uint32_t)comp->GetType()].push_back(comp);
            }
            gameobj = nullptr;
        }
    }

    // update render destroy pending array
    for (GameObject*& gameobj: g_render_destroy_pending) {
        if(!gameobj->IsRenderInitialized()) {
			assert(std::find(g_world_objects.begin(), g_world_objects.end(), gameobj) ==
				   g_world_objects.end());

            delete gameobj;
            gameobj = nullptr;
        }
    }

	{
		auto b = std::begin(g_render_init_pending);
		auto e = std::end(g_render_init_pending);
		g_render_init_pending.erase(
			std::remove_if(b, e, [](GameObject *o) { return o == nullptr; }), e);
	}
	{
		auto b = std::begin(g_render_destroy_pending);
		auto e = std::end(g_render_destroy_pending);
		g_render_destroy_pending.erase(
			std::remove_if(b, e, [](GameObject *o) { return o == nullptr; }), e);
	}

	// schedule render resource initialization for pending objects
	for (GameObject* gameobj: g_render_init_pending) {
        // TEMP: TODO: I know I can't touch GameObject in render thread
        // but I need to initalize its render state
        // make it shared object and take weak ref to it, then check if it is
        // still valid when render thread does it's job
	    ScheduleRenderCommand(rfc, [gameobj]() { gameobj->DoInitRenderResources(); });
	}

	for (GameObject *gameobj : g_render_destroy_pending) {
		ScheduleRenderCommand(rfc, [gameobj]() { gameobj->DoDeinitRenderResources(); });
	}
	//
}

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3 &ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj) {

    using el_t = std::pair<float, GameObject *>;
    for (auto &obj : g_world_objects) {
        if (!obj->GetMesh())
            continue;


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

        vec3 t = intersect_aabb_ray(obj->GetMesh()->aabb_, os_orig, os_dir);
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
