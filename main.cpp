#include "engine/utils/timing.h"
#include "engine/utils/gl_utils.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/camera.h"
#include "engine/utils/frustum.h"
#include "engine/utils/obj_loader.h"
#include "engine/gameos.hpp"

#include "renderer.h"
#include "shadow_renderer.h"
#include "particle_system.h"
#include "debug_renderer.h"
#include "deferred_renderer.h"

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

const uint32_t NUM_OBJECTS = 3;

bool g_render_initialized_hack = false;
bool g_update_simulation = true;

DWORD g_htexture = 0;
ShadowRenderPass* g_shadow_pass = nullptr;
uint32_t g_show_cascade_index = 2;
bool render_from_shadow_camera = false;

RenderMesh* g_fs_quad = 0;
RenderMesh* g_sphere = 0;

class FrustumObject;
FrustumObject* g_frustum_object = nullptr;

std::vector<PointLight> g_light_list;

struct DebugRenderObject {
    RenderMesh mesh_;
    mat4 transform_;
    vec4 colour;
};

struct SVD {
    vec3 pos;
    vec2 uv;
    vec3 normal;
};

struct QVD {
    vec2 pos;
    vec3 uv;
};

gosVERTEX_FORMAT_RECORD simple_vdecl[] =
{ 
    {0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0 },
    {1, 2, false, sizeof(SVD), offsetof(SVD, uv) , gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
    {2, 3, false, sizeof(SVD), offsetof(SVD, normal) , gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
};

gosVERTEX_FORMAT_RECORD pos_only_vdecl[] =
{ 
    {0, 3, false, sizeof(vec3), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
};

gosVERTEX_FORMAT_RECORD quad_vdecl[] =
{ 
    {0, 2, false, sizeof(QVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0 },
    //{1, 2, false, sizeof(QVD), offsetof(QVD, uv), gosVERTEX_ATTRIB_TYPE::FLOAT }
};

static HGOSVERTEXDECLARATION get_svd_vdecl() {
    static auto vdecl = gos_CreateVertexDeclaration(simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}

RenderMesh* CreateRenderMesh(const ObjFile* obj) {

        RenderMesh* ro = new RenderMesh();
        uint32_t* ib;
        ObjVertex* vb;
        uint32_t ib_count, vb_count;
        create_index_and_vertex_buffers(obj, &ib, &ib_count, &vb, &vb_count);

        static_assert(sizeof(SVD) == sizeof(ObjVertex), "Vetex structure sizez are different");
        ro->vdecl_ = get_svd_vdecl();

        ro->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                   gosBUFFER_USAGE::STATIC_DRAW,
                                   sizeof(uint32_t), ib_count, ib);
        ro->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                   gosBUFFER_USAGE::STATIC_DRAW, sizeof(ObjVertex),
                                   vb_count, vb);
        delete[] ib;
        delete vb;

        return ro;
}


RenderMesh* CreateCubeRenderMesh(const char* /*res*/) {

        constexpr const size_t NUM_VERT = 36; 
        constexpr const size_t NUM_IND = 36; 

        SVD vb[NUM_VERT];
        gen_cube_vb(&vb[0], NUM_VERT);

        RenderMesh* ro = new RenderMesh();

        uint16_t ib[NUM_IND];
        for(uint32_t i=0;i<NUM_IND;++i)
            ib[i] = i;

        ro->vdecl_ = gos_CreateVertexDeclaration(
            simple_vdecl,
            sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
        ro->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                   gosBUFFER_USAGE::STATIC_DRAW,
                                   sizeof(uint16_t), NUM_IND, ib);
        ro->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                   gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD),
                                   NUM_VERT, vb);

        return ro;
}

class GameObject;

DeferredRenderer g_deferred_renderer;

std::list<GameObject*> g_world_objects;
std::unordered_map<std::string, RenderMesh*> g_world_meshes;
std::unordered_map<std::string, DWORD> g_world_textures;

DWORD load_texture(const std::string& name) {
    DWORD tex_id = 0;
    auto tex_it = g_world_textures.find(name);
    if(tex_it!=g_world_textures.end()) {
        tex_id = tex_it->second;
    } else {
        std::string tex_fname = "data/textures/" + name + ".tga";
        tex_id = gos_NewTextureFromFile(gos_Texture_Detect, tex_fname.c_str());
        if(0 == tex_id) {
            printf("Failed to load: %s\n", tex_fname.c_str());
            tex_id = g_world_textures["default"];
        } else
            g_world_textures.insert(std::make_pair(name, tex_id));
    }
    return tex_id;
}

RenderMesh* load_mesh(const std::string mesh_name) {

    auto it = g_world_meshes.find(mesh_name);
    if(it!=g_world_meshes.end())
        return it->second;

    RenderMesh* mesh = nullptr;

    std::string fname = "data/meshes/" + mesh_name + ".obj";
    ObjFile* obj = load_obj_from_file(fname.c_str());
    if(!obj) {
        printf("Failed to load: %s\n", fname.c_str());
        mesh = g_world_meshes["cube"];
        assert(mesh);
    } else {
        mesh = CreateRenderMesh(obj);
        mesh->tex_id_ = load_texture(mesh_name);
    }
    g_world_meshes.insert(std::make_pair(mesh_name, mesh));

    return mesh;
}


class Renderable {
public:
    virtual void InitRenderResources() = 0;
    virtual ~Renderable() {}
};

class GameObject: public Renderable {
public:
    virtual void Update(float dt) = 0;
    virtual RenderMesh* GetMesh() const = 0;
    virtual mat4 GetTransform() const = 0;
    virtual ~GameObject() {}
};

class ParticleSystemObject: public GameObject {
    ParticleSystem* ps_;
    mat4 tr_;
public:
    static ParticleSystemObject* Create()
    {
        ParticleSystem* ps = new ParticleSystem;

        ParticleSystemObject* pso = new ParticleSystemObject;
        pso->ps_ = ps;
        pso->tr_ = mat4::identity();

        return pso;
    }

    void InitRenderResources() {

        ps_->AddEmitter(CreateStandardEmitter());
        ParticleSystemManager::Instance().Add(ps_);
    }

    virtual void Update(float /*dt*/) { }

    virtual RenderMesh* GetMesh() const { return nullptr; }
    virtual mat4 GetTransform() const { return tr_; }
    virtual mat4 SetTransform(const mat4& tr) { return tr_ = tr; }
    virtual ~ParticleSystemObject() {
        ParticleSystemManager::Instance().Remove(ps_);
        delete ps_;
    }
};

class FrustumObject: public GameObject {
    Frustum frustum_;
    RenderMesh* mesh_;

    public:

        static FrustumObject* Create(camera* pcam) {
            FrustumObject* o = new FrustumObject(pcam->view_, pcam->get_fov(), pcam->get_aspect(), pcam->get_near(), pcam->get_far());
            o->UpdateFrustum(pcam);
            return o;
        }

        FrustumObject(const mat4& view, float fov, float aspect, float nearv, float farv):
            mesh_(nullptr)
        {
        }

        ~FrustumObject() {
            DeinitRenderResources(); // dangerous
        }
        
        void Update(float dt) {
            //UpdateFrustum(view_, fov_, aspect_, nearv_, farv_);
        }

        void UpdateFrustum(camera* pcam)
        {
            mat4 view;
            pcam->get_view(&view);
            vec3 dir = view.getRow(2).xyz();
            vec3 right = view.getRow(0).xyz();
            vec3 up = view.getRow(1).xyz();
            vec4 pos = pcam->inv_view_ * vec4(0,0,0,1);

            frustum_.updateFromCamera(pos.xyz(), dir, right, up,
                                      pcam->get_fov(), pcam->get_aspect(),
                                      pcam->get_near(), pcam->get_far());

            // dangerous: we are on a game thread
            if(mesh_)
            {
                const int vb_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
                SVD vb[vb_size];
                Frustum::makeMeshFromFrustum(&frustum_, (char*)&vb[0], vb_size, (int)sizeof(SVD));

                for(int i=0;i<vb_size;++i)
                {
                    vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
                    vb[i].normal = normalize(vb[i].pos);
                }
                gos_UpdateBuffer(mesh_->vb_, vb, 0, vb_size * sizeof(SVD));
            }
            //
        }

        void InitRenderResources() {

            mesh_ = new RenderMesh();
            mesh_->vdecl_ = gos_CreateVertexDeclaration(
                simple_vdecl,
                sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));

            const int ib_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
            const int vb_size = ib_size;
            uint16_t ib[ib_size];
            SVD vb[vb_size];
            memset(&vb, 0, sizeof(vb));
            for(int i=0;i<ib_size;++i)
                ib[i] = i;

            mesh_->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                          gosBUFFER_USAGE::STATIC_DRAW,
                                          sizeof(uint16_t), ib_size, ib);

            Frustum::makeMeshFromFrustum(&frustum_, (char*)&vb[0], vb_size, (int)sizeof(SVD));

            for(int i=0;i<vb_size;++i)
            {
                vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
                vb[i].normal = normalize(vb[i].pos);
            }

            mesh_->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                          gosBUFFER_USAGE::DYNAMIC_DRAW,
                                          sizeof(SVD), vb_size, vb);
            mesh_->tex_id_ = g_htexture;

        }

        void DeinitRenderResources() {
            delete mesh_;
        }

        RenderMesh* GetMesh() const { return mesh_; }

        mat4 GetTransform() const { return mat4::identity(); }

};

class MeshObject: public GameObject {
public:
  typedef std::function<void(float dt, MeshObject *)> Updater_t;

private:
    std::string name_;
    std::string mesh_name_;
    RenderMesh* mesh_;

    vec3 scale_;
    vec3 rot_;
    vec3 pos_;

    MeshObject():mesh_(nullptr) {}

    Updater_t updater_;

public:
   static MeshObject* Create(const char* res) {
       static size_t obj_num = 0;
       MeshObject* obj = new MeshObject();
       obj->name_ = res;
       obj->name_ += std::to_string(obj_num++);
       obj->mesh_name_ = res;

       return obj;
   }

   void InitRenderResources()
   {
       //assert(IsOnRenderThread());
       mesh_ = load_mesh(mesh_name_);
   }

   // TODO: store and  return ref counted object
   // and test deferred deletion
   RenderMesh* GetMesh() const { return mesh_; }

   const vec3& GetPosition() const { return pos_; }
   vec3& GetPosition() { return pos_; }
   const char* GetName() { return name_.c_str(); } 

   void SetPosition(const vec3& pos) { pos_ = pos; }
   void SetRotation(const vec3& rot) { rot_ = rot; }
   void SetScale(const vec3& scale) { scale_ = scale; }
   void SetUpdater(Updater_t updater) { updater_ = updater; }

   void Update(float dt) {
       if (updater_)
           updater_(dt, this);
   }

   mat4 GetTransform() const {
       return translate(pos_) * rotateZXY4(rot_.x, rot_.y, rot_.z) *
              scale4(scale_.x, scale_.y, scale_.z);
   }
};

float get_random(float min, float max)
{
    float v = float(rand()%RAND_MAX) / float(RAND_MAX);
    return min + v * (max - min);
}

vec3 get_random(const vec3& minv, const vec3& maxv)
{
    return vec3(get_random(minv.x, maxv.x), get_random(minv.y, maxv.y),
                get_random(minv.z, maxv.z));
}

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

    g_camera.set_pos(vec3(0,15,0));
    g_camera.set_projection(45, SCREEN_W, SCREEN_H, 0.1f, 1000.0f);
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

    SDL_SetRelativeMouseMode(SDL_TRUE);
}

void __stdcall Deinit(void)
{
    DeleteRenderLists();
    delete g_fs_quad;

    delete g_shadow_pass;

    // destroy render meshes here

    for(auto tex_id: g_world_textures)
        gos_DestroyTexture(tex_id.second);

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();
    for(;it!=end;++it)
    {
        GameObject* go = *it;
        delete go;
    }

    printf("::Deinit\n");
}

void UpdateCamera(float dt)
{
    static float moveSpeedK = 50.0f;
    static float angularSpeedK = 0.25 * 3.1415f / 180.0f; // 0.25 degree per pixel

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

    //vec4 pos = g_camera.inv_view_ * vec4(0,0,0,1);
    //printf("cam pos: %f %f %f\n", pos.x, pos.y, pos.z);

    render_from_shadow_camera = gos_GetKeyStatus(KEY_O) ? true : false;
    g_show_cascade_index += gos_GetKeyStatus(KEY_C) == KEY_PRESSED ? 1 : 0;
    if(g_show_cascade_index>2)
        g_show_cascade_index = 0;

}

void __stdcall Update(void)
{
    static bool initialization_done = false;
    if(!initialization_done)
    {
        for (uint32_t i = 0; i < NUM_OBJECTS; ++i) {
            MeshObject *go = MeshObject::Create("N");
            vec3 base_pos = get_random(vec3(-15, 5, -15), vec3(15, 10, 15));
            go->SetPosition(base_pos);
            go->SetScale(get_random(vec3(1), vec3(2.5)));
            go->SetRotation(get_random(vec3(0), vec3(2.0f * 3.1415f)));
#if 1
            float start_time = (float)timing::ticks2ms(timing::gettickcount()) / 1000;
            const float amplitude = get_random(12.0f, 30.0f);
            const float phase = get_random(0.0f, 2.0*3.1415);
            go->SetUpdater(
                [base_pos, start_time, amplitude, phase](float dt, MeshObject *go) mutable {
                    vec3 p = go->GetPosition();
                    p.y = base_pos.y + amplitude * 0.5 * (sin(phase + start_time) + 1.0f);
                    start_time += dt;
#if DO_BAD_THING_FOR_TEST
                    go->SetPosition(get_random(vec3(-10), vec3(10)) +
                                    vec3(1000, 1000, 1000));
                // timing::sleep(100000);
#endif
                    go->SetPosition(p);
                });
#endif

            g_world_objects.push_back(go);
        }

        MeshObject* go = MeshObject::Create("column");
        go->SetPosition(vec3(0,0,0));
        go->SetScale(vec3(4, 8, 4));
        g_world_objects.push_back(go);

        go = MeshObject::Create("floor");
        go->SetPosition(vec3(0,0,0));
        go->SetScale(vec3(50, 1, 50));
        g_world_objects.push_back(go);

        camera loc_cam = g_camera;
        loc_cam.set_projection(45, Environment.drawableWidth, Environment.drawableHeight, 2.0f, 20.0f);
        FrustumObject* fo = FrustumObject::Create(&loc_cam);
        g_frustum_object = fo;
        g_world_objects.push_back(fo);

        ParticleSystemObject* pso = ParticleSystemObject::Create();
        g_world_objects.push_back(pso);
#if 0
        go = MeshObject::Create("minicooper");
        go->SetRotation(vec3(-90*3.1415f/180.0f,0.0f,0.0f));
        go->SetPosition(vec3(10,0,10));
        go->SetScale(vec3(0.1f));
        g_world_objects.push_back(go);
#endif
        // make vilage
        const float rot[] = {0, 150, 30, 90, 55};
        const float scales[] = {0.1f, 0.07f, 0.12f, 0.08f, 0.1f};
        const vec2 pos[] = {vec2(10,10), vec2(-10,-10), vec2(30, 35), vec2(10, -25), vec2(-30, 5)};
        for(int i=0; i < 5; ++i)
        {
            go = MeshObject::Create("single_room_building");
            go->SetRotation(vec3(0.0f, -rot[i]*3.1415f/180.0f, 0.0f));
            go->SetPosition(vec3(pos[i].x, 0.0f, pos[i].y));
            go->SetScale(vec3(scales[i]));
            g_world_objects.push_back(go);
        }

        // create some point lights
        for(int i=0; i<100;++i)
        {
            PointLight l;
            vec3 color = random_vec(0.f, 1.0f);
            float intensity = get_random(0.5f, 1.5f);
            l.color_ = vec4(color.x, color.y, color.z, intensity);
            l.radius_ = random(2.5f, 7.0f);
            vec3 pos = random_vec(vec3(-50.0f, 5.0f, -50.0f), vec3(50.0f, 15.0f, 50.0f));
            l.pos = pos;
            l.transform_ = translate(pos) * mat4::scale(vec3(l.radius_));
            g_light_list.push_back(l);
        }

        initialization_done = true;
    }

    RenderFrameContext* rfc = new RenderFrameContext();

    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    uint64_t dt = timing::ticks2ms(end_tick - start_tick);

    //printf("dt: %zd\n", dt);

    start_tick = timing::gettickcount();

    if(gos_GetKeyStatus(KEY_P) == KEY_PRESSED)
        g_update_simulation = !g_update_simulation;

    if(g_render_initialized_hack)
       UpdateCamera(dt*0.001f);

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();

    if(g_update_simulation)
    {
        ParticleSystemManager::Instance().Update(dt);
    
        for(;it!=end;++it)
        {
            GameObject* go = *it;
            go->Update(dt*0.001f);
        }
        it = g_world_objects.begin();
    }

    // prepare list of objects to render
    RenderList* frame_render_list = AcquireRenderList();

    char listidx_str[32] = {0};
    sprintf(listidx_str, "list_idx: %d", frame_render_list->GetId());
    rmt_BeginCPUSampleDynamic(listidx_str, 0);

    if(frame_render_list->GetCapacity() < g_world_objects.size())
        frame_render_list->ReservePackets(g_world_objects.size());

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

    if(g_sphere)
    {
        frame_render_list->ReservePackets(g_light_list.size());
        // add lights to debug render pass
        for(auto& l: g_light_list) {
            RenderPacket* rp = frame_render_list->AddPacket();
            rp->mesh_ = *g_sphere;
            rp->m_ = l.transform_ * mat4::scale(vec3(0.1f));
            rp->debug_color = vec4(l.color_.getXYZ(), 0.5f);
            rp->is_debug_pass = 1;
            rp->is_opaque_pass = 0;
            rp->is_render_to_shadow = 0;
            rp->is_transparent_pass = 0;
        }
    }

    rmt_EndCPUSample();

    // setup render frame context
    CSMInfo csm_info;
    fill_csm_frustums(&csm_info, &g_camera, &g_shadow_camera);
    rfc->frame_number_ = GetCurrentFrame();
    rfc->rl_ = frame_render_list;
    rfc->csm_info_ = csm_info;
    g_camera.get_view(&rfc->view_);
    g_camera.get_projection(&rfc->proj_);
    rfc->inv_proj_ = g_camera.inv_proj_;
    g_shadow_camera.get_view(&rfc->shadow_view_);
    rfc->shadow_inv_view_ = g_shadow_camera.inv_view_;
    rfc->point_lights_ = g_light_list;
    SetRenderFrameContext(rfc);
    //
    
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

public:

	void setup(const mat4& view, const mat4& proj)
	{
		view_ = view;
        proj_ = proj;
	}

    void set_shadow_params(const vec4 lightdir, const mat4 *shadow_matrices,
                           const DWORD *shadow_maps, const float *zfar) {
        shadow_matrices_ = shadow_matrices;
        shadow_maps_ = shadow_maps;
        lightdir_ = lightdir;
        zfar_ = zfar;
    }

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");

        gos_SetRenderState(gos_State_Texture, ro.tex_id_);
        gos_SetRenderState(gos_State_Filter, gos_FilterBiLinear);

        gos_SetRenderState(gos_State_Texture2, shadow_maps_[0]);
        gos_SetRenderState(gos_State_Texture3, shadow_maps_[1]);

		//gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
        
        mat4 wv = proj_ * view_;
        mat4 wvp = wv * rp.m_;
        mat4 vpshadow0 = shadow_matrices_[0];
        mat4 vpshadow1 = shadow_matrices_[1];
        float has_texture[] = { ro.tex_id_ ? 1.0f : 0.0f, 
                                (shadow_maps_[0] || shadow_maps_[1]) ? 1.0f : 0.0f, 
                                0.0f, 
                                0.0f };

		//gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		//gos_SetRenderMaterialParameterMat4(mat, "vpshadow_", (const float*)vpshadow);
		gos_SetRenderMaterialParameterMat4(mat, "wvpshadow0_", (const float*)(vpshadow0 * rp.m_));
		gos_SetRenderMaterialParameterMat4(mat, "wvpshadow1_", (const float*)(vpshadow1 * rp.m_));
		gos_SetRenderMaterialParameterFloat4(mat, "lightdir_", (const float*)lightdir_);
		gos_SetRenderMaterialParameterFloat4(mat, "has_texture", has_texture);
		gos_SetRenderMaterialParameterFloat4(mat, "z_far_", zfar_);

		gos_ApplyRenderMaterial(mat);

		// TODO: either use this or setMat4("wvp_", ...);
		//mat->setTransform(*wvp_);

        if (ro.ib_) {
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);
        } else if(ro.inst_vb_) {
    		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_);
        } else {
    		gos_RenderArray(ro.vb_, ro.vdecl_);
        }
    }
};

void render_fullscreen_quad(uint32_t tex_id)
{
	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    glViewport(0, 0, (GLsizei)Environment.drawableWidth, (GLsizei)Environment.drawableHeight);
    
    gos_SetRenderState(gos_State_Texture, tex_id);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial(tex_id ? "textured_quad" : "coloured_quad");

    float colour[4] = {1.0f,1.0f,1.0f,1.0f};
    if(!tex_id)
    {
        gos_SetRenderMaterialParameterFloat4(mat, "colour", colour);
    }

    gos_ApplyRenderMaterial(mat);

    gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_, g_fs_quad->vdecl_);

}

void __stdcall Render(void)
{
    static bool initialized = false;
    if(!initialized)
    {

        DWORD def_tex = gos_NewTextureFromFile(
            gos_Texture_Detect, "data/textures/texture_density.tga");
        assert(def_tex);
        g_world_textures.insert(std::make_pair("default", def_tex));

        RenderMesh* def = CreateCubeRenderMesh("cube");
        def->tex_id_ = def_tex;
        g_world_meshes.insert(std::make_pair("cube", def));

        // TEMP: TODO: I know I can't touch GameObject in render thread
        // but I need to initalize its render state
        std::list<GameObject*>::const_iterator it = g_world_objects.begin();
        std::list<GameObject*>::const_iterator end = g_world_objects.end();
        for(;it!=end;++it)
        {
            (*it)->InitRenderResources();
        }

        // init fullscreen quad geometry and shaders
        if(!g_fs_quad)
        {
            constexpr const size_t NUM_VERT = 4;
            constexpr const size_t NUM_IND = 6;

            QVD vb[NUM_VERT] = {{vec2(-1.0f, -1.0f)},
                                {vec2(-1.0f, 1.0f)},
                                {vec2(1.0f, -1.0f)},
                                {vec2(1.0f, 1.0f)}};
            uint16_t ib[NUM_IND] = { 0, 2, 3, 0, 3, 1} ;

            g_fs_quad = new RenderMesh();
            g_fs_quad->vdecl_ = gos_CreateVertexDeclaration(
                quad_vdecl,
                sizeof(quad_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
            g_fs_quad->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                              gosBUFFER_USAGE::STATIC_DRAW,
                                              sizeof(uint16_t), NUM_IND, ib);
            g_fs_quad->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                              gosBUFFER_USAGE::STATIC_DRAW,
                                              sizeof(QVD), NUM_VERT, vb);

            gos_AddRenderMaterial("coloured_quad");
            gos_AddRenderMaterial("textured_quad");
        }

        if(!g_sphere) {
            struct SVDAdapter {
                SVD* vb_ = nullptr;
                uint16_t* ib_ = nullptr;
                size_t vb_size_ = 0;
                size_t ib_size_ = 0;

                void allocate_vb(size_t size) {
                    delete vb_;
                    vb_ = new SVD[size];
                    vb_size_ = size;
                }
                void allocate_ib(size_t size) {
                    delete ib_;
                    ib_ = new uint16_t[size];
                    ib_size_ = size;
                }
                void p(unsigned int i, vec3 p) {
                    vb_[i].pos = p;
                }
                void n(unsigned int i, vec3 n) {
                    vb_[i].normal = n;
                }
                void uv(unsigned int i, vec2 uv) {
                    vb_[i].uv = uv;
                }
                void i(unsigned int i, uint16_t idx) {
                    ib_[i] = idx;
                }
            };
            SVDAdapter svd_adapter;
            generate_sphere(svd_adapter, 5);
            g_sphere = new RenderMesh();
            g_sphere->vdecl_ = get_svd_vdecl();
            g_sphere->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                              gosBUFFER_USAGE::STATIC_DRAW,
                                              sizeof(uint16_t), svd_adapter.ib_size_, svd_adapter.ib_);
            g_sphere->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                              gosBUFFER_USAGE::STATIC_DRAW,
                                              sizeof(SVD), svd_adapter.vb_size_, svd_adapter.vb_);

            gos_AddRenderMaterial("debug");
        }

        g_shadow_pass = new ShadowRenderPass();
        if(!g_shadow_pass->Init(1024, 2))
        {
            SPEW(("SHADOWS", "Failed to init shadow pass"));
        }

        gos_AddRenderMaterial("directional_shadow");
        gos_AddRenderMaterial("particle");

        g_deferred_renderer.Init(SCREEN_W, SCREEN_H);

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


    if(0)
    {
        camera loc_cam = g_camera;
        loc_cam.set_projection(45.0f, Environment.drawableWidth, Environment.drawableHeight, 4.0f, 20.0f);
        g_frustum_object->UpdateFrustum(&loc_cam);
    }

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

    shape_renderer.set_shadow_params(lightdir, cascade_matrices, cascade_shadow_maps, csm_info.zfar_); 

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();

    char rfc_info[128] = {0};
    sprintf(rfc_info, "rfc: %d rl: %d", rfc->frame_number_, rfc->rl_->GetId());
    rmt_BeginCPUSampleDynamic(rfc_info, 0);

#if defined(FORWARD_RENDERING)
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_Culling, render_from_shadow_camera? gos_Cull_CW : gos_Cull_CCW);
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
    Environment.screenWidth = SCREEN_W;
    Environment.screenHeight = SCREEN_H;

    Environment.InitializeGameEngine = Init;
    Environment.DoGameLogic = Update;
    Environment.TerminateGameEngine = Deinit;
    Environment.UpdateRenderers = Render;
}




