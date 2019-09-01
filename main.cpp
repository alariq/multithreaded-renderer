#include "engine/utils/timing.h"
#include "engine/utils/gl_utils.h"
#include "engine/utils/camera.h"
#include "engine/utils/frustum.h"
#include "engine/gameos.hpp"

#include "renderer.h"

#include "Remotery/lib/Remotery.h"
#include <cstdlib>
#include <cstddef>
#include <string>
#include <functional>


extern int RendererGetNumBufferedFrames();
extern int RendererGetCurrentFrame();
extern int GetCurrentFrame();
extern void SetRenderFrameContext(void* rfc);
extern void* GetRenderFrameContext();

#if defined(DO_TESTS)
extern void test_fixed_block_allocator();
#endif

const uint32_t NUM_OBJECTS = 3;

bool g_render_initialized_hack = false;

DWORD g_htexture = 0;
ShadowRenderPass* g_shadow_pass = nullptr;
bool render_from_shadow_camera = false;

RenderMesh* g_fs_quad = 0;

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
    {0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT },
    {1, 2, false, sizeof(SVD), offsetof(SVD, uv) , gosVERTEX_ATTRIB_TYPE::FLOAT},
    {2, 3, false, sizeof(SVD), offsetof(SVD, normal) , gosVERTEX_ATTRIB_TYPE::FLOAT},
};

gosVERTEX_FORMAT_RECORD pos_only_vdecl[] =
{ 
    {0, 3, false, sizeof(vec3), 0, gosVERTEX_ATTRIB_TYPE::FLOAT },
};

gosVERTEX_FORMAT_RECORD quad_vdecl[] =
{ 
    {0, 2, false, sizeof(QVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT },
    //{1, 2, false, sizeof(QVD), offsetof(QVD, uv), gosVERTEX_ATTRIB_TYPE::FLOAT }
};


RenderMesh* CreateCubeRenderMesh(const char* /*res*/) {

        constexpr const size_t NUM_VERT = 36; 
        constexpr const size_t NUM_IND = 36; 

        SVD vb[NUM_VERT];
        gen_cube_vb(&vb[0], NUM_VERT);

        RenderMesh* ro = new RenderMesh();

        uint16_t ib[NUM_IND];
        for(uint32_t i=0;i<NUM_IND;++i)
            ib[i] = i;

        ro = new RenderMesh();

	    ro->vdecl_ = gos_CreateVertexDeclaration(simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
		ro->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), NUM_IND, ib);
		ro->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD), NUM_VERT, vb);

        return ro;
}

class Renderable {
public:
    virtual void InitRenderResources() = 0;
    virtual ~Renderable() {}
};

class GameObject: public Renderable {
public:
    virtual void Update(float dt, float current_time) = 0;
    virtual RenderMesh* GetMesh() const = 0;
    virtual mat4 GetTransform() const = 0;
    virtual ~GameObject() {}
};

class FrustumObject: public GameObject {
    Frustum frustum_;
    RenderMesh* mesh_;

    mat4 view_;
    float fov_;
    float aspect_;
    float nearv_;
    float farv_;

    public:

        static FrustumObject* Create(const mat4& view, float fov, float aspect, float nearv, float farv) {
            FrustumObject* o = new FrustumObject(view, fov, aspect, nearv, farv);
            o->UpdateFrustum(view, fov, aspect, nearv, farv);
            return o;
        }

        FrustumObject(const mat4& view, float fov, float aspect, float nearv, float farv):
            mesh_(nullptr), view_(view), fov_(fov), aspect_(aspect), nearv_(nearv), farv_(farv)
        {
        }

        ~FrustumObject() {
            DeinitRenderResources(); // dangerous
        }
        
        void Update(float dt, float current_time) {
            //UpdateFrustum(view_, fov_, aspect_, nearv_, farv_);
        }

        void UpdateFrustum(const mat4& view, float fov, float aspect, float nearv, float farv)
        {
            view_ = view;
            fov_ = fov;
            aspect_ = aspect;
            nearv_ = nearv;
            farv_ = farv;

            frustum_.updateFromCamera(view_, fov_, aspect_, nearv_, farv_);

            // dangerous: we are on a game thread
            if(mesh_)
            {
                const int vb_size = Frustum::NUM_FRUSTUM_PLANES * 6;
                SVD vb[vb_size];
                Frustum::makeMeshFromFrustum(&frustum_, (char*)&vb[0], vb_size, (int)sizeof(SVD));
                gos_UpdateBuffer(mesh_->vb_, vb, 0, vb_size * sizeof(SVD));
            }
            //
        }

        void InitRenderResources() {

            mesh_ = new RenderMesh();
	        mesh_->vdecl_ = gos_CreateVertexDeclaration(simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));

            const int ib_size = Frustum::NUM_FRUSTUM_PLANES * 6;
            const int vb_size = ib_size;
            uint16_t ib[ib_size];
            SVD vb[vb_size];
            memset(&vb, 0, sizeof(vb));
            for(int i=0;i<ib_size;++i)
                ib[i] = i;

            mesh_->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), ib_size, ib);

            frustum_.updateFromCamera(view_, fov_, aspect_, nearv_, farv_);
            Frustum::makeMeshFromFrustum(&frustum_, (char*)&vb[0], vb_size, (int)sizeof(SVD));

            for(int i=0;i<vb_size;++i)
            {
                vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
                vb[i].normal = normalize(vb[i].pos);
            }

            mesh_->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW, sizeof(SVD), vb_size, vb);
            mesh_->tex_id_ = g_htexture;

        }

        void DeinitRenderResources() {
            delete mesh_;
        }

        RenderMesh* GetMesh() const { return mesh_; }

        mat4 GetTransform() const { return mat4::identity(); }

};

class CubeGameObject: public GameObject {
public:
    typedef std::function<void(float dt, float current_time, CubeGameObject*)> Updater_t;
private:
    std::string name_;
    RenderMesh* mesh_;

    vec3 scale_;
    vec3 rot_;
    vec3 pos_;

    CubeGameObject():mesh_(nullptr) {}

    Updater_t updater_;

public:
   static CubeGameObject* Create(const char* res) {
       static size_t obj_num = 0;
       CubeGameObject* obj = new CubeGameObject();
       obj->name_ = res;
       obj->name_ += std::to_string(obj_num++);

       return obj;
   }

   void InitRenderResources()
   {
       //aseert(IsOnRenderThread());
       mesh_ = CreateCubeRenderMesh(name_.c_str());
       mesh_->tex_id_ = g_htexture;
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

   void Update(float dt, float current_time) { if(updater_) updater_(dt, current_time, this); }

   mat4 GetTransform() const {
       return translate(pos_) * rotateZXY4(rot_.x, rot_.y, rot_.z) * scale4(scale_.x, scale_.y, scale_.z);
   }
};

std::list<GameObject*> g_world_objects;

float get_random(float min, float max)
{
    float v = float(rand()%RAND_MAX) / float(RAND_MAX);
    return min + v * (max - min);
}

vec3 get_random(const vec3& minv, const vec3& maxv)
{
    return vec3(
            get_random(minv.x, maxv.x),
            get_random(minv.y, maxv.y),
            get_random(minv.z, maxv.z)
            );
}

camera g_camera;
camera g_shadow_camera;

void __stdcall Init(void)
{
    printf("::Init\n");

#if defined(DO_TESTS)
    test_fixed_block_allocator();
#endif

    g_camera.set_pos(vec3(0,15,0));
    g_camera.update(0.0f);


    vec3 light_dir = normalize(vec3(-1.0,-1.0,-1.0));

    vec3 up = vec3(0,1,0);

    vec3 right = normalize(cross(up, light_dir));
    up = normalize(cross(light_dir, right));

    mat4 shadow_view = mat4::identity();
    // negative light dir by convention
    camera::compose_view_matrix(&shadow_view, right, up, -light_dir, -light_dir*50.0f);

    g_shadow_camera.set_ortho_projection(-30, 30, 30, -30, 0.1f, 200.0f);
    g_shadow_camera.set_view(shadow_view);
    //g_shadow_camera.update(0);

    SDL_SetRelativeMouseMode(SDL_TRUE);
}

void __stdcall Deinit(void)
{
    DeleteRenderLists();
    delete g_fs_quad;

    // can't delere render resources here because renderer is already destroyed
    // TODO: come up with correct shutdown procedure
    //delete g_shadow_pass;
    //gos_DestroyTexture(g_htexture);

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

    float aspect = Environment.drawableWidth / Environment.drawableHeight;
    g_camera.set_projection(fov, aspect, 0.1f, 1000.0f);

    g_camera.update(dt);

    render_from_shadow_camera = gos_GetKeyStatus(KEY_O) ? true : false;

}

void __stdcall Update(void)
{
    static bool initialization_done = false;
    if(!initialization_done)
    {
        for(uint32_t i=0; i<NUM_OBJECTS; ++i)
        {
            CubeGameObject* go = CubeGameObject::Create("N");
            vec3 base_pos = get_random(vec3(-15, 5, -15), vec3(15, 10, 15));
            go->SetPosition(base_pos);
            go->SetScale(get_random(vec3(1), vec3(2.5)));
            go->SetRotation(get_random(vec3(0), vec3(2.0f*3.1415f)));
#if 1
            go->SetUpdater([base_pos](float dt, float current_time, CubeGameObject* go) {

                vec3 p = go->GetPosition();
                p.y = base_pos.y + base_pos.y*sin(current_time*0.001);
                #if DO_BAD_THING_FOR_TEST
                    go->SetPosition(get_random(vec3(-10), vec3(10)) + vec3(1000, 1000, 1000));
                    //timing::sleep(100000);
                #endif
                go->SetPosition(p);
            });
#endif

            g_world_objects.push_back(go);
        }

        CubeGameObject* go = CubeGameObject::Create("column");
        go->SetPosition(vec3(0,0,0));
        go->SetScale(vec3(4, 8, 4));
        g_world_objects.push_back(go);

        go = CubeGameObject::Create("floor");
        go->SetPosition(vec3(0,0,0));
        go->SetScale(vec3(20, 1, 30));
        g_world_objects.push_back(go);

        mat4 vm;
        g_camera.get_view(&vm);
        float aspect = 800.0f/ 600.0f; //Environment.drawableWidth / Environment.drawableHeight;
        FrustumObject* fo = FrustumObject::Create(vm, aspect, 45.0f * 3.1415f / 180.0f, 2.0f, 20.0f);
        g_world_objects.push_back(fo);

        initialization_done = true;
    }

    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    uint64_t dt = timing::ticks2ms(end_tick - start_tick);

    //printf("dt: %zd\n", dt);

    start_tick = timing::gettickcount();

    if(g_render_initialized_hack)
       UpdateCamera(dt*0.001f);

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();
    for(;it!=end;++it)
    {
        GameObject* go = *it;
        go->Update(dt*0.001f, end_tick);
    }


    // prepare list of objects to render
    it = g_world_objects.begin();
    
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

        // maybe instead create separate render thread render objects or make all render object always live on render thread
        if(go->GetMesh()) // if initialized
        {
            RenderPacket* rp = frame_render_list->AddPacket();

            rp->mesh_ = go->GetMesh();
            rp->m_ = go->GetTransform();
#if DO_BAD_THING_FOR_TEST
            rp->go_ = go;
#endif
        }
    }

    rmt_EndCPUSample();

    RenderFrameContext* rfc = new RenderFrameContext();
    rfc->frame_number_ = GetCurrentFrame();
    rfc->rl_ = frame_render_list;
    SetRenderFrameContext(rfc);
}

class ShapeRenderer {

	mat4 view_;
	mat4 proj_;
    DWORD shadow_map_;
    mat4 shadow_matrix_;
    vec4 lightdir_;

public:

	void setup(const mat4& view, const mat4& proj)
	{
		view_ = view;
        proj_ = proj;
	}

    void set_shadow_params(const mat4& shadow_matrix, vec4 lightdir, DWORD shadow_map)
    {
        shadow_matrix_ = shadow_matrix;
        shadow_map_ = shadow_map;
        lightdir_ = lightdir;
    }

	void render(const RenderPacket& rp)
	{
        RenderMesh& ro = *rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");

        gos_SetRenderState(gos_State_Texture, ro.tex_id_);
        gos_SetRenderState(gos_State_Filter, gos_FilterBiLinear);

        gos_SetRenderState(gos_State_Texture2, shadow_map_);

		//gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
        
        mat4 wv = proj_ * view_;
        mat4 wvp = wv * rp.m_;
        mat4 vpshadow = shadow_matrix_;
        float has_texture[] = { ro.tex_id_ ? 1.0f : 0.0f, shadow_map_ ? 1.0f : 0.0f, 0.0f, 0.0f };

		//gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		//gos_SetRenderMaterialParameterMat4(mat, "vpshadow_", (const float*)vpshadow);
		gos_SetRenderMaterialParameterMat4(mat, "wvpshadow_", (const float*)(vpshadow * rp.m_));
		gos_SetRenderMaterialParameterFloat4(mat, "lightdir_", (const float*)lightdir_);
		gos_SetRenderMaterialParameterFloat4(mat, "has_texture", has_texture);

		gos_ApplyRenderMaterial(mat);

		// TODO: either use this or setMat4("wvp_", ...);
		//mat->setTransform(*wvp_);

		gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);

	}

};

void render_fullscreen_quad(uint32_t tex_id)
{
	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    glViewport(0, 0, (GLsizei)Environment.drawableWidth, (GLsizei)Environment.drawableHeight);
    
    gos_SetRenderState(gos_State_Texture, tex_id);

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
        g_htexture = gos_NewTextureFromFile(gos_Texture_Detect, "data/textures/a_bturretcontrol.tga");
        assert(g_htexture);

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

            QVD vb[NUM_VERT] = { {vec2(-1.0f, -1.0f)}, {vec2(-1.0f, 1.0f)}, {vec2(1.0f, -1.0f)}, {vec2(1.0f, 1.0f)} };
            uint16_t ib[NUM_IND] = { 0, 2, 3, 0, 3, 1} ;

            g_fs_quad = new RenderMesh();
            g_fs_quad->vdecl_ = gos_CreateVertexDeclaration(quad_vdecl, sizeof(quad_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
            g_fs_quad->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), NUM_IND, ib);
            g_fs_quad->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(QVD), NUM_VERT, vb);

            gos_AddRenderMaterial("coloured_quad");
            gos_AddRenderMaterial("textured_quad");
        }


        g_shadow_pass = new ShadowRenderPass();
        if(!g_shadow_pass->Init(1024))
        {
            SPEW(("SHADOWS", "Failed to init shadow pass"));
        }

        gos_AddRenderMaterial("directional_shadow");


        initialized = true;

        g_render_initialized_hack = true;
    }

    RenderFrameContext* rfc = (RenderFrameContext*)GetRenderFrameContext();
    assert(rfc);
    assert(rfc->frame_number_ == RendererGetCurrentFrame());

    // render shadows first
    g_shadow_pass->Render(&g_shadow_camera, rfc->rl_->GetRenderPackets());

    gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    glViewport(0, 0, (GLsizei)Environment.drawableWidth, (GLsizei)Environment.drawableHeight);

    gos_SetRenderState(gos_State_TextureAddress, gos_TextureWrap);
    
    mat4 view_mat, proj_mat;
    if(render_from_shadow_camera)
    {
        g_shadow_camera.get_view(&view_mat);
        g_shadow_camera.get_projection(&proj_mat);
    }
    else
    {
        g_camera.get_view(&view_mat);
        g_camera.get_projection(&proj_mat);
    }

    ShapeRenderer shape_renderer;
    shape_renderer.setup(view_mat, proj_mat);
    mat4 shadow_view_m;
    mat4 shadow_proj_m;
    vec4 lightdir;
    lightdir = -shadow_view_m.getRow(2);
    g_shadow_camera.get_view(&shadow_view_m);
    g_shadow_camera.get_projection(&shadow_proj_m);
    shape_renderer.set_shadow_params(shadow_proj_m * shadow_view_m, lightdir, g_shadow_pass->GetShadowMap());

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    char rfc_info[128] = {0};
    sprintf(rfc_info, "rfc: %d rl: %d", rfc->frame_number_, rfc->rl_->GetId());
    rmt_BeginCPUSampleDynamic(rfc_info, 0);

    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
#if DO_BAD_THING_FOR_TEST
        // test: get transform on render thread which can't be done (because it is changing on game thread)
        // also cache miss on attempt to call function
        rp.m_ = rp.go_->GetTransform();
#endif

        shape_renderer.render(rp);
    }

    rmt_EndCPUSample();

    ReleaseRenderList(rfc->rl_);
    delete rfc;

    render_fullscreen_quad(g_shadow_pass->GetShadowMap());
    
}

void GetGameOSEnvironment(const char* cmdline)
{
    (void)cmdline;
    Environment.screenWidth = 800;
    Environment.screenHeight = 600;

    Environment.InitializeGameEngine = Init;
    Environment.DoGameLogic = Update;
    Environment.TerminateGameEngine = Deinit;
    Environment.UpdateRenderers = Render;
}




