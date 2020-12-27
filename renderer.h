#pragma once 

#include "engine/utils/gl_utils.h"
#include "engine/utils/ts_queue.h"
#include "engine/utils/camera.h"
#include "engine/utils/frustum.h"
#include "engine/utils/intersection.h" //aabb
#include "engine/gameos.hpp"

#include <vector>
#include <atomic>
#include <functional>

struct SVD {
    vec3 pos;
    vec2 uv;
    vec3 normal;
};

struct QVD {
    vec2 pos;
    vec3 uv;
};

HGOSVERTEXDECLARATION get_svd_vdecl();
HGOSVERTEXDECLARATION get_pos_only_vdecl();
HGOSVERTEXDECLARATION get_quad_vdecl();

// uncomment to see shadowing artefacts because of game threads transforms used in render thread
//#define DO_BAD_THING_FOR_TEST 1

struct RenderMesh {
    HGOSBUFFER				vb_;
    HGOSBUFFER				ib_;
    HGOSBUFFER				inst_vb_;
    HGOSVERTEXDECLARATION	vdecl_;
    HGOSRENDERMATERIAL      mat_;
    uint32_t                num_instances;
    uint32_t                tex_id_;
    gosPRIMITIVETYPE        prim_type_;
    AABB                    aabb_;
    uint32_t                vb_first_;
    uint32_t                vb_count_;
};

struct PointLight {
    vec4 color_; // w - intensity
    float radius_;
    mat4 transform_;
    vec3 pos;
};

struct RenderPacket {
    RenderMesh mesh_;
    mat4 m_;
    vec4 debug_color;
	uint32_t id_;
    // material, other params
#if DO_BAD_THING_FOR_TEST
    class GameObject* go_;
#endif
    uint32_t is_render_to_shadow: 1;
    uint32_t is_transparent_pass: 1;
    uint32_t is_opaque_pass: 1;
    uint32_t is_debug_pass: 1;
    uint32_t is_selection_pass: 1;
    uint32_t is_gizmo_pass: 1;
};

class NonCopyable {
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable(NonCopyable&&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
protected:
    NonCopyable() = default;
};

typedef std::vector<RenderPacket> RenderPacketList_t;
class RenderList: public NonCopyable {
    std::atomic_int ref_count;
    int id_;
    RenderPacketList_t packets_;
public:
    RenderList():ref_count(0) {
        static int id = 0;
        id_ = id++;
    }

    void Acquire() { int old = ref_count.fetch_add(1); (void)old; assert(old==0); }
    void Release() { int old = ref_count.fetch_sub(1); (void)old; assert(old==1); }
    bool IsAcquired() { return ref_count == 1; }
    // for debugging purposes
    int GetId() const { return id_; }

    size_t GetCapacity() { return packets_.capacity(); }

    void ReservePackets(size_t count) {
        packets_.reserve(count);
    }
    
    RenderPacket* AddPacket() {
        //assert(packets_.size() + 1 <= packets_.capacity()); // just to ensure that we do not reallocate
        packets_.emplace_back(RenderPacket());
		RenderPacket* p = &packets_[packets_.size()-1];
		memset(p, 0, sizeof(RenderPacket));
		return p;
    }

    RenderPacketList_t& GetRenderPackets() { return packets_; }

};

RenderList* AcquireRenderList();
void ReleaseRenderList(RenderList* rl);
void DeleteRenderLists();

const int MAX_SHADOW_CASCADES = 4;

struct CSMInfo {
    uint32_t num_cascades_;
    Frustum fr_[MAX_SHADOW_CASCADES];
    //GLuint depth_textures_[MAX_SHADOW_CASCADES];
    mat4 shadow_vp_[MAX_SHADOW_CASCADES];
    float zfar_[MAX_SHADOW_CASCADES]; // in proj space
};

struct RenderFrameContext {
    RenderList* rl_;
    int frame_number_;
    CSMInfo csm_info_;
    std::vector<std::function<void(void)>> commands_;

    //TODO: for each view, setup its parameters, fbo, obj lists etc...
    mat4 view_;
    mat4 proj_;
    mat4 inv_proj_;
    mat4 inv_view_;
    mat4 shadow_view_;
    mat4 shadow_inv_view_;

    // point lights that passed frustum culling
    std::vector<PointLight> point_lights_;
};

void ScheduleRenderCommand(RenderFrameContext *rfc, std::function<void(void)>&& );
