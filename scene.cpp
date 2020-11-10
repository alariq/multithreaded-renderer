#include "scene.h"
#include "particle_system.h"
#include "sph.h"
#include "res_man.h"
#include "obj_model.h"
#include "renderer.h"

#include "utils/camera.h"
#include "utils/math_utils.h"
#include "utils/timing.h"
#include <list>

typedef std::list<GameObject*> ObjList_t;
static ObjList_t g_world_objects;
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
#if 0
    const uint32_t NUM_OBJECTS = 3;
    for (uint32_t i = 0; i < NUM_OBJECTS; ++i) {
        MeshObject *go = MeshObject::Create("N");
        vec3 base_pos = random_vec(vec3(-15, 5, -15), vec3(15, 10, 15));
        auto* tr_comp = go->GetComponent<TransformComponent>();
        tr_comp->SetPosition(base_pos);
        tr_comp->SetScale(random_vec(vec3(1), vec3(2.5)));
        tr_comp->SetRotation(random_vec(vec3(0), vec3(2.0f * 3.1415f)));
#if 1
        float start_time =
            (float)timing::ticks2ms(timing::gettickcount()) / 1000;
        const float amplitude = random(12.0f, 30.0f);
        const float phase = random(0.0f, 2.0f * 3.1415f);
        go->SetUpdater([base_pos, start_time, amplitude,
                        phase](float dt, MeshObject *go) mutable {
            auto* tc = go->GetComponent<TransformComponent>();
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

        g_world_objects.push_back(go);
    }

    MeshObject* go = MeshObject::Create("column");
    auto* tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 0, 0));
    tc->SetScale(vec3(4, 8, 4));
    g_world_objects.push_back(go);
#endif 
    MeshObject* go = MeshObject::Create("floor");
    auto* tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, -1, 0));
    tc->SetScale(vec3(50, 1, 50));
    g_world_objects.push_back(go);
#if 0
    go = MeshObject::Create("axes");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 20, 0));
    tc->SetScale(vec3(10, 10, 10));
    g_world_objects.push_back(go);

    go = MeshObject::Create("torus");
    tc = go->GetComponent<TransformComponent>();
    tc->SetPosition(vec3(0, 30, 0));
    tc->SetScale(vec3(10, 10, 10));
    g_world_objects.push_back(go);

    camera loc_cam = *cam;
    loc_cam.set_projection(45, Environment.drawableWidth,
                           Environment.drawableHeight, 2.0f, 20.0f);
    FrustumObject *fo = FrustumObject::Create(&loc_cam);
    g_world_objects.push_back(fo);

    ParticleSystemObject *pso = ParticleSystemObject::Create();
    g_world_objects.push_back(pso);
#endif
    SPHSceneObject* sph = SPHSceneObject::Create(vec2(5, 10), 500, vec3(0, 0, 0));
    sph->SetRadius(0.1f);
    initialize_particle_positions(sph);
    g_world_objects.push_back(sph);

#if 0
    // make vilage
    const float rot[] = {0, 150, 30, 90, 55};
    const float scales[] = {0.1f, 0.07f, 0.12f, 0.08f, 0.1f};
    const vec2 pos[] = {vec2(10, 10), vec2(-10, -10), vec2(30, 35),
                        vec2(10, -25), vec2(-30, 5)};
    for (int i = 0; i < 5; ++i) {
        go = MeshObject::Create("single_room_building");
        auto* t = go->GetComponent<TransformComponent>();
        t->SetRotation(vec3(0.0f, -rot[i] * 3.1415f / 180.0f, 0.0f));
        t->SetPosition(vec3(pos[i].x, 0.0f, pos[i].y));
        t->SetScale(vec3(scales[i]));
        g_world_objects.push_back(go);
    }
#endif
    // create some point lights
    for (int i = 0; i < 100; ++i) {
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

    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();
    for (; it != end; ++it) {
        GameObject *gameobj = *it;

        // TEMP: TODO: I know I can't touch GameObject in render thread
        // but I need to initalize its render state
        // make it shader object and take weak ref to it, then check if it is
        // still valid when render thread does it's job
        ScheduleRenderCommand(rfc, [gameobj]() { gameobj->InitRenderResources(); });
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

void scene_update(const camera *cam, const float dt) {
    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();

    ParticleSystemManager::Instance().Update(dt);

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

void scene_render_update(struct RenderFrameContext *rfc) {

    RenderList *frame_render_list = rfc->rl_;

    if (frame_render_list->GetCapacity() < g_world_objects.size())
        frame_render_list->ReservePackets(g_world_objects.size());

    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();

    for (; it != end; ++it) {
        const GameObject *go = *it;

        // check if visible
        // ...

        // maybe instead create separate render thread render objects or make
        // all render object always live on render thread
        if (go->GetMesh()) // if initialized
        {
            RenderPacket *rp = frame_render_list->AddPacket();
            const auto* tc = go->GetComponent<TransformComponent>();

            rp->mesh_ = *go->GetMesh();
            rp->m_ = tc ? tc->GetTransform() : mat4::identity();
			rp->id_ = go->GetId();
            rp->is_opaque_pass = 1;
            rp->is_render_to_shadow = 1;
            rp->is_transparent_pass = 0;
            rp->is_debug_pass = 0;
            rp->is_selection_pass = 1;
#if DO_BAD_THING_FOR_TEST
            rp->go_ = go;
#endif
        }

        go->AddRenderPackets(rfc);

    }


    rfc->point_lights_ = g_light_list;
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
