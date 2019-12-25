#pragma once

#include "renderer.h"

#include "engine/gameos.hpp"
#include<vector>
#include<string>
#include<mutex>
#include<algorithm>

struct ParticleInstVDecl {
    float lifetime_;
    vec3 pos_;
};

struct ParticleVDecl {
    vec2 uv;
};

class ParticleEmitterInterface {
public:

    enum Type: uint32_t {
        Line,
        Rectangle,
        Cube,
        Circle,
        Sphere
    };

    virtual void Update(const float dt) = 0;
    //virtual void FillVertexBuffer(ParticleVDecl* vb, size_t& count) = 0;
    virtual size_t GetCount() const = 0; 
    virtual HGOSRENDERMATERIAL GetMaterial() const = 0;
    virtual void AddRenderPacket(struct RenderFrameContext* rfc) = 0;
    virtual void InitRenderResources() = 0;
    virtual void DestroyRenderResources() = 0;

};

ParticleEmitterInterface* CreateStandardEmitter();

class ParticleSystem {
public:
    void Update(const float dt);
    void Render(struct RenderFrameContext *rfc);
    void AddEmitter(ParticleEmitterInterface* e) {
        emitters_.push_back(e);
    }
    void InitRenderResources();
    void DestroyRenderResources();
private:
    std::vector<ParticleEmitterInterface*> emitters_;
};

class ParticleSystemManager {
    std::mutex init_rr;
public:
    void Add(ParticleSystem* ps)
    {
        std::lock_guard<std::mutex> l(init_rr);
        need_initialize_.push_back(ps);
    }
    void Remove(ParticleSystem* ps)
    {
        std::lock_guard<std::mutex> l(init_rr);

        auto it = std::find(ps_list_.begin(), ps_list_.end(), ps);
        if(it != ps_list_.end()) {
            ps_list_.erase(it);
        } else {
            auto it =
                std::find(need_initialize_.begin(), need_initialize_.end(), ps);
            if(it != need_initialize_.end())
                need_initialize_.erase(it);
        }
    }

    void Update(const float dt);
    void Render(struct RenderFrameContext *rfc);

    static ParticleSystemManager& Instance() {
        static ParticleSystemManager psm;
        return psm;
    }

    void InitRenderResources()
    {
        std::lock_guard<std::mutex> l(init_rr);
        for(auto ps: need_initialize_)
            ps->InitRenderResources();

        ps_list_.insert(ps_list_.end(), need_initialize_.begin(),
                        need_initialize_.end());
        need_initialize_.clear();
    }

    void DestroyRenderResources();

private:
    std::vector<ParticleSystem*> need_initialize_;
    std::vector<ParticleSystem*> ps_list_;
};

void RenderParticles(const RenderPacketList_t& rpl, const mat4& view, const mat4& proj);

