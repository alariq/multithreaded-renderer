#include "shadow_renderer.h"

#include "renderer.h"

#include "engine/utils/gl_utils.h"
#include "engine/utils/gl_fbo.h"
#include "engine/utils/frustum.h"

#include <cfloat>

mat4 applyCropMatrix(const Frustum &f, const mat4& shadow_modelview)
{
	float maxX = -FLT_MAX;
    float maxY = -FLT_MAX;
	float maxZ;
    float minX =  FLT_MAX;
    float minY =  FLT_MAX;
	float minZ;

	vec4 transf;	
	
	// find the z-range of the current frustum as seen from the light
	// in order to increase precision
	
	// note that only the z-component is need and thus
	// the multiplication can be simplified
	// transf.z = shad_modelview[2] * f.point[0].x + shad_modelview[6] * f.point[0].y + shad_modelview[10] * f.point[0].z + shad_modelview[14];
    const vec3* fr_points = f.getPoints();
    
	transf = shadow_modelview * vec4(fr_points[0], 1.0f);
	minZ = transf.z;
	maxZ = transf.z;
	for(int i=1; i<Frustum::kNUM_FRUSTUM_POINTS; i++)
	{
		transf = shadow_modelview * vec4(fr_points[i], 1.0f);
		if(transf.z > maxZ) maxZ = transf.z;
		if(transf.z < minZ) minZ = transf.z;
	}

#if 0
	// make sure all relevant shadow casters are included
	// note that these here are dummy objects at the edges of our scene
	for(int i=0; i<NUM_OBJECTS; i++)
	{
		transf = shadow_modelview*vec4f(obj_BSphere[i].center, 1.0f);
		if(transf.z + obj_BSphere[i].radius > maxZ) maxZ = transf.z + obj_BSphere[i].radius;
	//	if(transf.z - obj_BSphere[i].radius < minZ) minZ = transf.z - obj_BSphere[i].radius;
	}

#endif
    // include 8 points at the corners of scene AABB (TODO: provide real correct AABB)
	for(int i=0; i<8; i++)
	{
        vec3 pt = vec3(i%2?-200.0f:200.0f, ((i>>1)%2)?-200.0f:200.0f, ((i>>2)%2)?-200.0f:200.0f); 
		transf = shadow_modelview * vec4(pt.x, pt.y, pt.z, 1.0f);
		if(transf.z > maxZ) maxZ = transf.z;
	    if(transf.z < minZ) minZ = transf.z;
	}

    mat4 shad_proj = orthoMatrix(-1, 1, 1, -1, -maxZ, -minZ, false);
    mat4 shad_mvp = shad_proj * shadow_modelview;

	// set the projection matrix with the new z-bounds
	// note the inversion because the light looks at the neg. z axis
	// gluPerspective(LIGHT_FOV, 1.0, maxZ, minZ); // for point lights
    // in case of directional light
    //glOrtho(-1.0, 1.0, -1.0, 1.0, -maxZ, -minZ);

	// find the extends of the frustum slice as projected in light's homogeneous coordinates
	for(int i=0; i<8; i++)
	{
		transf = shad_mvp*vec4(fr_points[i], 1.0f);

		transf.x /= transf.w;
		transf.y /= transf.w;

		if(transf.x > maxX) maxX = transf.x;
		if(transf.x < minX) minX = transf.x;
		if(transf.y > maxY) maxY = transf.y;
		if(transf.y < minY) minY = transf.y;
	}

#if 1
    // TODO: constrain by real scene AABB (it can be smaller than frustum)
    vec3 sc_min(-200,-200,-200);    
    vec3 sc_max(200,200,200);    

	float sc_maxX = -FLT_MAX;
    float sc_maxY = -FLT_MAX;
	float sc_maxZ = FLT_MAX;
    float sc_minX =  FLT_MAX;
    float sc_minY =  FLT_MAX;
	float sc_minZ =  -FLT_MAX;

	for(int i=0; i<8; i++)
	{
        vec3 pt = vec3(i%2?-200.0f:200.0f, ((i>>1)%2)?-200.0f:200.0f, ((i>>2)%2)?-200.0f:200.0f); 
		transf = shad_mvp*vec4(pt.x, pt.y, pt.z, 1.0f);

		transf.x /= transf.w;
		transf.y /= transf.w;

		if(transf.x > sc_maxX) sc_maxX = transf.x;
		if(transf.x < sc_minX) sc_minX = transf.x;
		if(transf.y > sc_maxY) sc_maxY = transf.y;
		if(transf.y < sc_minY) sc_minY = transf.y;
		if(transf.z > sc_maxZ) sc_maxZ = transf.z;
		if(transf.z < sc_minZ) sc_minZ = transf.z;
	}

    // intersect
    maxX = sc_maxX < maxX ? sc_maxX : maxX;
    minX = sc_minX > minX ? sc_minX : minX;
    maxY = sc_maxY < maxY ? sc_maxY : maxY;
    minY = sc_minY > minY ? sc_minY : minY;
    maxZ = sc_maxZ < maxZ ? sc_maxZ : maxZ;
    minZ = sc_minZ > minZ ? sc_minZ : minZ;
#endif

    mat4 new_proj = orthoMatrix(minX, maxX, maxY, minY, -maxZ, -minZ, false);

	//return minZ;
    return new_proj;
}

void fill_csm_frustums(CSMInfo* csm_info, const struct camera* cam, const struct camera* shadow_cam)
{
    // create 2 cascades: first close up with Zn=CamZn Zf=10 and second Zn=10 Zf = CamZf;
    assert(csm_info);
    csm_info->num_cascades_ = 2;

    const float zn[] = {cam->get_near(), 20.0f};
    const float zf[] = {20.0f, cam->get_far()};
    constexpr const size_t num_cascades = sizeof(zn)/sizeof(zn[0]);
    static_assert(num_cascades < MAX_SHADOW_CASCADES, "Number of shadow cascades exceeds maximum");

    mat4 view_m;
    cam->get_view(&view_m);
    mat4 proj_m;
    cam->get_projection(&proj_m);

    mat4 shadow_view_m;
    shadow_cam->get_view(&shadow_view_m);

    vec3 dir = view_m.getRow(2).xyz();
    vec3 right = view_m.getRow(0).xyz();
    vec3 up = view_m.getRow(1).xyz();
    vec4 pos = cam->get_inv_view() * vec4(0,0,0,1);

    csm_info->num_cascades_ = num_cascades;
    for(size_t i=0; i<num_cascades;++i)
    {
        Frustum& fr = csm_info->fr_[i];

        float cascade_near = zn[i];
        float cascade_far = zf[i];

        fr.updateFromCamera(pos.xyz(), dir, right, up , cam->get_fov(), cam->get_aspect(), cascade_near, cascade_far);

        const mat4 shadow_proj = applyCropMatrix(fr, shadow_view_m);
        csm_info->shadow_vp_[i] = shadow_proj * shadow_view_m;

        // view -> projection + normalize to 0..1
        csm_info->zfar_[i] = 0.5f * (proj_m * vec4(0.0f, 0.0f, cascade_far, 1.0f)).z / cascade_far + 0.5f;
    }
}

bool ShadowRenderPass::Init(const uint32_t size, const int num_cascades)
{
    width_ = size;
    height_ = size;
    num_cascades_ = num_cascades;
    use_pcf_ = true;

    // not a real constraint, but gos_NewRenderTarget accepts only one dimension parameter :-/
    assert(width_ == height_); 
    assert(num_cascades < MAX_SHADOW_CASCADES);

    gos_TextureAddressMode addr = gos_TextureClampToBorder;
    gos_FilterMode filter = use_pcf_ ? gos_FilterBiLinear : gos_FilterNone;
    gos_TextureCompareMode cmp_mode = use_pcf_ ? gos_TextureCompareRefToTexture : gos_TextureCompareNone;
    gos_CompareFunc cmp_func = gos_Cmp_Less;

    smp_shadow_ =
        gos_CreateTextureSampler(addr, addr, addr, filter, filter, filter,
                                 false, cmp_mode, cmp_func, vec4(0, 0, 0, 0));

    glGenFramebuffers(num_cascades, &fbo_);
    for(int i=0; i<num_cascades;++i)
    {
        char name[256];
        sprintf(name, "shadow_map%d", i);
        gos_depth_texture_[i] = gos_NewRenderTarget(gos_Texture_Depth, name, width_);
        depth_texture_id_[i] = gos_TextureGetNativeId(gos_depth_texture_[i]);

        glBindTexture(GL_TEXTURE_2D, depth_texture_id_[i]);
        //setSamplerParams(TT_2D, TAM_CLAMP_TO_EDGE, TFM_NEAREST);
        
        //setSamplerParams(TT_2D, TAM_CLAMP_TO_EDGE, TFM_LINEAR);
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, use_pcf_ ? GL_LINEAR : GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, use_pcf_ ? GL_LINEAR : GL_NEAREST);
        if(use_pcf_)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);

        //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, );
        glBindTexture(GL_TEXTURE_2D, 0);

        //glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        //glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_id_, 0);
        //glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    return true;
}

mat4 ShadowRenderPass::Render(const struct CSMInfo *csm_info,
                              const RenderPacketList_t &rpl) {

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_);

    assert(num_cascades_ <= csm_info->num_cascades_);

    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);

    glEnable(GL_POLYGON_OFFSET_FILL);

    for(uint32_t i = 0; i < num_cascades_; ++i)
    {
        // make values be dependent on a cascade
        static float a = 1.0f;
        static float b = 4096.0f;
        glPolygonOffset(a, b);

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_id_[i], 0);

        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        checkFramebufferStatus();

        glClear(GL_DEPTH_BUFFER_BIT);

        gos_SetRenderViewport(0, 0, width_, height_);

        glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

        // TODO: do occlusion check against each cascade!

        RenderPacketList_t::const_iterator it = rpl.begin();
        RenderPacketList_t::const_iterator end = rpl.end();

        gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

        for(;it!=end;++it)
        {
            const RenderPacket& rp = (*it);

            if(!rp.is_render_to_shadow)
                continue;

            const RenderMesh& ro = rp.mesh_;

            HGOSRENDERMATERIAL mat = gos_getRenderMaterial("directional_shadow");

            mat4 wvp = csm_info->shadow_vp_[i] * rp.m_;

            gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);

            gos_ApplyRenderMaterial(mat);
            if(ro.ib_)
                gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_, ro.prim_type_);
            else
                gos_RenderArray(ro.vb_, ro.vdecl_, ro.prim_type_);
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDrawBuffer(GL_BACK);

    glDisable(GL_POLYGON_OFFSET_FILL);

    return csm_info->shadow_vp_[0];
}

ShadowRenderPass::~ShadowRenderPass()
{
    glDeleteFramebuffers(num_cascades_, &fbo_);
    for(uint32_t i = 0; i < num_cascades_; ++i)
    {
	    gos_DestroyTexture(gos_depth_texture_[i]);
    }
}
