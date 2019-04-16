#include "utils/timing.h"
#include "utils/gl_utils.h"
#include "engine/gameos.hpp"
#include <cstddef>

void __stdcall Init(void)
{
    printf("::Init\n");
}

void __stdcall Deinit(void)
{
    printf("::Deinit\n");
}

void __stdcall Update(void)
{
    static uint64_t start_tick = timing::gettickcount();

    uint64_t end_tick = timing::gettickcount();
    uint64_t dt = timing::ticks2ms(end_tick - start_tick);

    printf("dt: %d\n", dt);

    start_tick = timing::gettickcount();

    static int i = 0;
    //if(i > 10)
    //    gos_TerminateApplication();
    i++;

}

struct RenderObject {

    HGOSBUFFER				vb_;
    HGOSBUFFER				ib_;
    HGOSVERTEXDECLARATION	vdecl_;
    HGOSRENDERMATERIAL      mat_;
    uint32_t                tex_id_;
    mat4                    world_mat_;
};

class ShapeRenderer {

	mat4 view_;
	mat4 proj_;

public:

	void setup(const mat4& view, const mat4& proj)
	{
		view_ = view;
        proj_ = proj;
	}

	void render(RenderObject& ro)
	{
		gos_SetRenderState(gos_State_Texture, ro.tex_id_);


		HGOSRENDERMATERIAL mat = gos_getRenderMaterial("simple");
		gos_SetRenderMaterialParameterMat4(mat, "world_", (const float*)ro.world_mat_);
        
        mat4 wvp = proj_ * view_ * ro.world_mat_;
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);

		gos_ApplyRenderMaterial(mat);

		// TODO: either use this or setMat4("wvp_", ...);
		//mat->setTransform(*wvp_);

		gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);

	}

};

struct SVD {
    vec3 pos;
    vec2 uv;
    vec3 normal;
 //   SVD(const vec3& p, const vec2& uvuv, const vec3& n):
   //     pos(p), uv(uvuv), normal(n) {}

};

gosVERTEX_FORMAT_RECORD simple_vdecl[] =
{ 
    {0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT },
    {1, 2, false, sizeof(SVD), offsetof(SVD, uv) , gosVERTEX_ATTRIB_TYPE::FLOAT},
    {2, 3, false, sizeof(SVD), offsetof(SVD, normal) , gosVERTEX_ATTRIB_TYPE::FLOAT},
};


RenderObject* g_ro = 0;

void __stdcall Render(void)
{
    if(!g_ro)
    {

        constexpr const size_t NUM_VERT = 36; 
        constexpr const size_t NUM_IND = 36; 

        SVD vb[NUM_VERT];
        gen_cube_vb(&vb[0], NUM_VERT);

        uint16_t ib[NUM_VERT];
        for(int i=0;i<NUM_VERT;++i)
            ib[i] = i;

        g_ro = new RenderObject();

	    g_ro->vdecl_ = 
            gos_CreateVertexDeclaration(simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
		g_ro->ib_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t), NUM_IND, ib);
		g_ro->vb_ = 
            gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD), NUM_VERT, vb);

        g_ro->world_mat_ = mat4::identity();
    }

	gos_SetRenderViewport(0, 0, Environment.drawableWidth, Environment.drawableHeight);
    
    float aspect = Environment.drawableWidth / Environment.drawableHeight;

    gos_SetRenderState(gos_State_TextureAddress, gos_TextureWrap);

	mat4 proj_mat = frustumProjMatrix(-aspect*0.5f, aspect*0.5f, -.5f, .5f, 1.0f, 100.0f);

    static float angle = 0.0f;
    mat4 view_mat = mat4::translation(vec3(0, 0, -8)) * mat4::rotationY(angle);

    angle += 0.05f;

    ShapeRenderer shape_renderer;
    shape_renderer.setup(view_mat, proj_mat);
    shape_renderer.render(*g_ro);

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




