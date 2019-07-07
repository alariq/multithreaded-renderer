#pragma once 

#include "engine/utils/gl_utils.h"
#include "engine/utils/ts_queue.h"
#include "engine/utils/camera.h"
#include "engine/gameos.hpp"

#include <list>
#include <atomic>

// uncomment to see shadowing artefacts because of game threads transforms used in render thread
//#define DO_BAD_THING_FOR_TEST 1

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

class NonCopyable {
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
protected:
    NonCopyable()= default;
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
        size_t old_size = packets_.capacity();
        packets_.reserve(old_size + count);
    }
    
    // allocates "count" amount of packets and returns pointer to memory
    RenderPacket* AddPacket() {
        assert(packets_.size() + 1 <= packets_.capacity()); // just to ensure that we do not reallocate
        packets_.emplace_back(RenderPacket());
        return &packets_[packets_.size()-1];
    }

    RenderPacketList_t& GetRenderPackets() { return packets_; }

};

RenderList* AcquireRenderList();
void ReleaseRenderList(RenderList* rl);
void DeleteRenderLists();

struct RenderFrameContext {
    RenderList* rl_;
    int frame_number_;
};


class ShadowRenderPass {

    GLuint fbo_;
    DWORD gos_depth_texture_;
    GLuint depth_texture_id_;
    GLuint width_;
    GLuint height_;

public:
    bool Init(uint32_t size); 
    void Render(const struct camera* shadow_camera, const RenderPacketList_t& rpl);

    uint32_t GetShadowMap() { return gos_depth_texture_; }

    ~ShadowRenderPass();
    
};

