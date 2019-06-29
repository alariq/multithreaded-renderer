#include "engine/utils/timing.h"
#include "engine/utils/gl_utils.h"
#include "engine/gameos.hpp"

#include "Remotery/lib/Remotery.h"
#include <cstdlib>
#include <cstddef>
#include <string>
#include <list>

extern int RendererGetNumBufferedFrames();
extern int RendererGetCurrentFrame();
extern int GetCurrentFrame();

#if defined(DO_TESTS)
extern void test_fixed_block_allocator();
#endif

#define DO_BAD_THING_FOR_TEST 1

const uint32_t NUM_OBJECTS = 1000;


struct SVD {
    vec3 pos;
    vec2 uv;
    vec3 normal;
 //   SVD(const vec3& p, const vec2& uvuv, const vec3& n):
   //     pos(p), uv(uvuv), normal(n) {}

};

struct QVD {
    vec2 pos;
};

gosVERTEX_FORMAT_RECORD simple_vdecl[] =
{ 
    {0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT },
    {1, 2, false, sizeof(SVD), offsetof(SVD, uv) , gosVERTEX_ATTRIB_TYPE::FLOAT},
    {2, 3, false, sizeof(SVD), offsetof(SVD, normal) , gosVERTEX_ATTRIB_TYPE::FLOAT},
};

gosVERTEX_FORMAT_RECORD quad_vdecl[] =
{ 
    {0, 2, false, sizeof(QVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT }
};

struct RenderMesh {
    HGOSBUFFER				vb_;
    HGOSBUFFER				ib_;
    HGOSVERTEXDECLARATION	vdecl_;
    HGOSRENDERMATERIAL      mat_;
    uint32_t                tex_id_;
};

struct RenderPacket {
    RenderMesh* mesh_;
    mat4 m_;
    // material, other params
#if DO_BAD_THING_FOR_TEST
    class GameObject* go_;
#endif
};

RenderMesh* CreateRenderMesh(const char* /*res*/) {

        constexpr const size_t NUM_VERT = 36; 
        constexpr const size_t NUM_IND = 36; 

        SVD vb[NUM_VERT];
        gen_cube_vb(&vb[0], NUM_VERT);

        RenderMesh* ro = new RenderMesh();

        uint16_t ib[NUM_IND];
        for(uint32_t i=0;i<NUM_IND;++i)
            ib[i] = i;

        ro = new RenderMesh();

	    ro->vdecl_ = 
            gos_CreateVertexDeclaration(simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
		ro->ib_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), NUM_IND, ib);
		ro->vb_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD), NUM_VERT, vb);

        //ro->world_mat_ = mat4::identity();

        return ro;
}

class GameObject {
    std::string name_;
    RenderMesh* mesh_;

    vec3 scale_;
    vec3 rot_;
    vec3 pos_;

    GameObject():mesh_(nullptr) {}
public:
   static GameObject* Create(const char* res) {
       static size_t obj_num = 0;
       GameObject* obj = new GameObject();
       obj->name_ = res;
       obj->name_ += std::to_string(obj_num++);

       return obj;
   }

   void InitRenderResources()
   {
       //aseert(IsOnRenderThread());
       mesh_ = CreateRenderMesh(name_.c_str());
   }

   // TODO: store and  return ref counted object
   // and test deferred deletion
   RenderMesh* GetMesh() const { return mesh_; }

   const vec3& GetPosition() const { return pos_; }
   vec3& GetPosition() { return pos_; }

   void SetPosition(const vec3& pos) { pos_ = pos; }
   void SetRotation(const vec3& rot) { rot_ = rot; }
   void SetScale(const vec3& scale) { scale_ = scale; }

   mat4 GetTransform() const {
       return translate(pos_) * rotateZXY4(rot_.x, rot_.y, rot_.z) * scale4(scale_.x, scale_.y, scale_.z);
   }
};

std::list<GameObject*> g_world_objects;

typedef std::list<RenderPacket> RenderPacketList_t;
RenderPacketList_t* g_render_packets;

float get_random(float min, float max)
{
    float v = float(rand()%RAND_MAX) / float(RAND_MAX);
    return min + v * (max - min);
}

vec3 get_random(const vec3& minv, const vec3 maxv)
{
    return vec3(
            get_random(minv.x, maxv.x),
            get_random(minv.y, maxv.y),
            get_random(minv.z, maxv.z)
            );
}

void __stdcall Init(void)
{
    printf("::Init\n");

#if defined(DO_TESTS)
    test_fixed_block_allocator();
#endif

    int count = RendererGetNumBufferedFrames() + 1;
    g_render_packets = new RenderPacketList_t[count];
}

void __stdcall Deinit(void)
{
    printf("::Deinit\n");
}

void __stdcall Update(void)
{
    static bool initialization_done = false;
    if(!initialization_done)
    {
        for(uint32_t i=0; i<NUM_OBJECTS; ++i)
        {
            GameObject* go = GameObject::Create("N");
            go->SetPosition(get_random(vec3(-10), vec3(10)));
            go->SetScale(get_random(vec3(0.1), vec3(1)));
            go->SetRotation(get_random(vec3(0), vec3(2.0f*3.1415f)));

            g_world_objects.push_back(go);
        }
        initialization_done = true;
    }

    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    uint64_t dt = timing::ticks2ms(end_tick - start_tick);

    printf("dt: %zd\n", dt);

    start_tick = timing::gettickcount();

    static int i = 0;
    //if(i > 10)
    //    gos_TerminateApplication();
    i++;

    std::list<GameObject*>::const_iterator it = g_world_objects.begin();
    std::list<GameObject*>::const_iterator end = g_world_objects.end();
    for(;it!=end;++it)
    {
        vec3 p = (*it)->GetPosition();
        p.y = p.y + 0.1*sin(end_tick*0.001);
#if DO_BAD_THING_FOR_TEST
        (*it)->SetPosition(get_random(vec3(-10), vec3(10)) + vec3(1000, 1000, 1000));
        //timing::sleep(100000);
#endif

        (*it)->SetPosition(p);   
    }


    // prepare list of objects to render
    it = g_world_objects.begin();
    const int list_idx = GetCurrentFrame() % (RendererGetNumBufferedFrames()+1);
    RenderPacketList_t& current_render_list = g_render_packets[list_idx];

    char listidx_str[32] = {0};
    sprintf(listidx_str, "list_idx: %d", list_idx);
    rmt_BeginCPUSampleDynamic(listidx_str, 0);

    int counter = 0;
    for(;it!=end;++it)
    {
        GameObject* go = *it;

        // chek if visible
        // ...

        RenderPacket rp;
        // maybe instead create separate render thread render objects or make all render object always live on render thread
        if(go->GetMesh()) // if initialized
        {
            rp.mesh_ = go->GetMesh();
            rp.m_ = go->GetTransform();
#if DO_BAD_THING_FOR_TEST
            rp.go_ = go;
#endif
            current_render_list.push_back(rp);
        }

        counter++;
    }

    rmt_EndCPUSample();

}

class ShapeRenderer {

	mat4 view_;
	mat4 proj_;

public:

	void setup(const mat4& view, const mat4& proj)
	{
		view_ = view;
        proj_ = proj;
	}

	void render(const RenderPacket& rp)
	{
        RenderMesh& ro = *rp.mesh_;

		gos_SetRenderState(gos_State_Texture, ro.tex_id_);


		HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");
		//gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)rp.m_);
        
        mat4 wvp = proj_ * view_ * rp.m_;
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);

		gos_ApplyRenderMaterial(mat);

		// TODO: either use this or setMat4("wvp_", ...);
		//mat->setTransform(*wvp_);

		gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);

	}

};

RenderMesh* g_ro = 0;
RenderMesh* g_fs_quad = 0;

void render_fullscreen_quad()
{
    if(!g_fs_quad)
    {
        constexpr const size_t NUM_VERT = 4;
        constexpr const size_t NUM_IND = 6; 

        QVD vb[NUM_VERT] = { {vec2(-1.0f, -1.0f)}, {vec2(-1.0f, 1.0f)}, {vec2(1.0f, -1.0f)}, {vec2(1.0f, 1.0f)} };

        uint16_t ib[NUM_IND] = { 0, 2, 3, 0, 3, 1} ;

        g_fs_quad = new RenderMesh();

	    g_fs_quad->vdecl_ = 
            gos_CreateVertexDeclaration(quad_vdecl, sizeof(quad_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
		g_fs_quad->ib_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), NUM_IND, ib);
		g_fs_quad->vb_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD), NUM_VERT, vb);

        gos_AddRenderMaterial("coloured_quad");
    }

	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    
	//mat4 proj_mat = mat4::identity();
    //mat4 view_mat = mat4::identity();


    gos_SetRenderState(gos_State_Texture, 0);


    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("coloured_quad");

    float colour[4] = {1.0f,1.0f,1.0f,1.0f};
    gos_SetRenderMaterialParameterFloat4(mat, "colour", colour);

    gos_ApplyRenderMaterial(mat);

    gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_, g_fs_quad->vdecl_);

}

void __stdcall Render(void)
{
    static bool initialized = false;
    if(!initialized)
    {
        g_ro = CreateRenderMesh("");   

        // TEMP: TODO: I know I can;t touch thse in this thread
        std::list<GameObject*>::const_iterator it = g_world_objects.begin();
        std::list<GameObject*>::const_iterator end = g_world_objects.end();
        for(;it!=end;++it)
        {
            (*it)->InitRenderResources();
        }
        initialized = true;
    }

	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    
    float aspect = Environment.drawableWidth / Environment.drawableHeight;

    gos_SetRenderState(gos_State_TextureAddress, gos_TextureWrap);

	mat4 proj_mat = frustumProjMatrix(-aspect*0.5f, aspect*0.5f, -.5f, .5f, 1.0f, 100.0f);

    static float angle = 0.0f;
    mat4 view_mat = mat4::translation(vec3(0, 0, -8));// * mat4::rotationY(angle);

    angle += 0.05f;

    ShapeRenderer shape_renderer;
    shape_renderer.setup(view_mat, proj_mat);

    RenderPacket rp;
    rp.mesh_ = g_ro;
    rp.m_ = mat4::identity();
    shape_renderer.render(rp);

    
    const int list_idx = RendererGetCurrentFrame() % (RendererGetNumBufferedFrames()+1);
    RenderPacketList_t& current_render_list = g_render_packets[list_idx];

    RenderPacketList_t::const_iterator it = current_render_list.begin();
    RenderPacketList_t::const_iterator end = current_render_list.end();
    int counter = 0;

    char listidx_str[32] = {0};
    sprintf(listidx_str, "list_idx: %d", list_idx);
    rmt_BeginCPUSampleDynamic(listidx_str, 0);

    for(;it!=end;)
    {
        /*const*/ RenderPacket rp = (*it);
#if DO_BAD_THING_FOR_TEST
        // test: get transform on render thread which can't be done (becaus it is changing on game thread)
        // also cache miss on attempt to call function
        rp.m_ = rp.go_->GetTransform();
#endif

        shape_renderer.render(rp);

        RenderPacketList_t::const_iterator todel = it;
        it++;
        //current_render_list.erase(todel);
        counter++;
    }

    current_render_list.clear();

    rmt_EndCPUSample();

    render_fullscreen_quad();
    
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




