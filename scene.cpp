#include "scene.h"
#include "particle_system.h"
#include "res_man.h"
#include "obj_model.h"
#include "renderer.h"

#include "utils/camera.h"
#include "utils/math_utils.h"
#include "utils/timing.h"
#include <list>

static std::list<GameObject*> g_world_objects;
static std::vector<PointLight> g_light_list;

void initialize_scene(const camera* cam, class RenderFrameContext* rfc) {

    const uint32_t NUM_OBJECTS = 3;
    for (uint32_t i = 0; i < NUM_OBJECTS; ++i) {
        MeshObject *go = MeshObject::Create("N");
        vec3 base_pos = random_vec(vec3(-15, 5, -15), vec3(15, 10, 15));
        go->SetPosition(base_pos);
        go->SetScale(random_vec(vec3(1), vec3(2.5)));
        go->SetRotation(random_vec(vec3(0), vec3(2.0f * 3.1415f)));
#if 1
        float start_time =
            (float)timing::ticks2ms(timing::gettickcount()) / 1000;
        const float amplitude = random(12.0f, 30.0f);
        const float phase = random(0.0f, 2.0f * 3.1415f);
        go->SetUpdater([base_pos, start_time, amplitude,
                        phase](float dt, MeshObject *go) mutable {
            vec3 p = go->GetPosition();
            p.y =
                base_pos.y + amplitude * 0.5 * (sin(phase + start_time) + 1.0f);
            start_time += dt;
#if DO_BAD_THING_FOR_TEST
            go->SetPosition(random_vec(vec3(-10), vec3(10)) +
                            vec3(1000, 1000, 1000));
            // timing::sleep(100000);
#endif
            go->SetPosition(p);
        });
#endif

        g_world_objects.push_back(go);
    }

    MeshObject *go = MeshObject::Create("column");
    go->SetPosition(vec3(0, 0, 0));
    go->SetScale(vec3(4, 8, 4));
    g_world_objects.push_back(go);

    go = MeshObject::Create("floor");
    go->SetPosition(vec3(0, 0, 0));
    go->SetScale(vec3(50, 1, 50));
    g_world_objects.push_back(go);

    camera loc_cam = *cam;
    loc_cam.set_projection(45, Environment.drawableWidth,
                           Environment.drawableHeight, 2.0f, 20.0f);
    FrustumObject *fo = FrustumObject::Create(&loc_cam);
    g_world_objects.push_back(fo);

    ParticleSystemObject *pso = ParticleSystemObject::Create();
    g_world_objects.push_back(pso);

    // make vilage
    const float rot[] = {0, 150, 30, 90, 55};
    const float scales[] = {0.1f, 0.07f, 0.12f, 0.08f, 0.1f};
    const vec2 pos[] = {vec2(10, 10), vec2(-10, -10), vec2(30, 35),
                        vec2(10, -25), vec2(-30, 5)};
    for (int i = 0; i < 5; ++i) {
        go = MeshObject::Create("single_room_building");
        go->SetRotation(vec3(0.0f, -rot[i] * 3.1415f / 180.0f, 0.0f));
        go->SetPosition(vec3(pos[i].x, 0.0f, pos[i].y));
        go->SetScale(vec3(scales[i]));
        g_world_objects.push_back(go);
    }

    // create some point lights
    for (int i = 0; i < 100; ++i) {
        PointLight l;
        vec3 color = random_vec(0.f, 1.0f);
        float intensity = random(0.5f, 1.5f);
        l.color_ = vec4(color.x, color.y, color.z, intensity);
        l.radius_ = random(2.5f, 7.0f);
        vec3 pos =
            random_vec(vec3(-50.0f, 5.0f, -50.0f), vec3(50.0f, 15.0f, 50.0f));
        l.pos = pos;
        l.transform_ = translate(pos) * mat4::scale(vec3(l.radius_));
        g_light_list.push_back(l);
    }

    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();
    for (; it != end; ++it) {
        GameObject* go = *it;

        // TEMP: TODO: I know I can't touch GameObject in render thread
        // but I need to initalize its render state
        // make it shader object and take weak ref to it, then check if it is
        // still valid when render thread does it's job
        ScheduleRenderCommand(rfc, [go]() { go->InitRenderResources(); });
    }
}

void scene_update(const camera* cam, const float dt)
{
    std::list<GameObject *>::const_iterator it = g_world_objects.begin();
    std::list<GameObject *>::const_iterator end = g_world_objects.end();

    ParticleSystemManager::Instance().Update(dt);

    for (; it != end; ++it) {
        GameObject *go = *it;
        go->Update(dt * 0.001f);

        // if object is frustum object.... and we wnt to update it
        if (0) {
            camera loc_cam = *cam;
            loc_cam.set_projection(45.0f, Environment.drawableWidth, Environment.drawableHeight, 4.0f, 20.0f);
            ((FrustumObject*)go)->UpdateFrustum(&loc_cam);
        }
    }
}


void scene_render_update(class RenderFrameContext* rfc) {

    RenderList* frame_render_list = rfc->rl_;

    if(frame_render_list->GetCapacity() < g_world_objects.size())
        frame_render_list->ReservePackets(g_world_objects.size());

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();

    for(;it!=end;++it)
    {
        GameObject* go = *it;

        // check if visible
        // ...

        // maybe instead create separate render thread render objects or make
        // all render object always live on render thread
        if(go->GetMesh()) // if initialized
        {
            RenderPacket* rp = frame_render_list->AddPacket();

            rp->mesh_ = *go->GetMesh();
            rp->m_ = go->GetTransform();
            rp->is_opaque_pass = 1;
            rp->is_render_to_shadow = 1;
            rp->is_transparent_pass = 0;
            rp->is_debug_pass = 0;
#if DO_BAD_THING_FOR_TEST
            rp->go_ = go;
#endif
        }
    }

    RenderMesh* sphere = res_man_load_mesh("sphere");
    assert(sphere);
    {
        frame_render_list->ReservePackets(g_light_list.size());
        // add lights to debug render pass
        for(auto& l: g_light_list) {
            RenderPacket* rp = frame_render_list->AddPacket();
            rp->mesh_ = *sphere;
            rp->m_ = l.transform_ * mat4::scale(vec3(0.1f));
            rp->debug_color = vec4(l.color_.getXYZ(), 0.5f);
            rp->is_debug_pass = 1;
            rp->is_opaque_pass = 0;
            rp->is_render_to_shadow = 0;
            rp->is_transparent_pass = 0;
        }
    }

    rfc->point_lights_ = g_light_list;
}



void finalize_scene() {

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();
    for(;it!=end;++it)
    {
        GameObject* go = *it;
        delete go;
    }
}
