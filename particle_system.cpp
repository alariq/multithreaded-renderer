#include "particle_system.h"
#include "renderer.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/vec.h"
#include <cmath>
#include <cassert>

namespace {

gosVERTEX_FORMAT_RECORD particle_vdecl[] = {
    // quad vb with uvs
    {0, 2, false, sizeof(ParticleVDecl), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},

    // instance data: stream 1
    {1, 1, false, sizeof(ParticleInstVDecl), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
    {2, 3, false, sizeof(ParticleInstVDecl), 4, gosVERTEX_ATTRIB_TYPE::FLOAT, 1},

};

struct EmitterGeometry {

    struct LineEmitterGeometry {
        vec3 p0;
        vec3 p1;
    };

    struct CircleEmitterGeometry {
        vec2 center;
        float radius;
    };

    struct SphereEmitterGeometry {
        vec3 center;
        float radius;
    };

    struct RectangleEmitterGeometry {
        vec2 left;
        vec2 top;
        vec2 right;
        vec2 bottom;
    };

    struct BoxEmitterGeometry {
        vec3 center;
        vec3 sides;
    };

    ParticleEmitterInterface::Type type_;

    union {
        LineEmitterGeometry lep_;
        CircleEmitterGeometry cep_;
        SphereEmitterGeometry sep_;
        RectangleEmitterGeometry rep_;
        BoxEmitterGeometry bep_;
    };
};

typedef void(*generator_t)(const EmitterGeometry& , vec3* , size_t );

void generate_position_line(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    const EmitterGeometry::LineEmitterGeometry& lep = ep.lep_;
    for(uint32_t i=0; i < count; ++i) {
        pos[i] = random_vec(lep.p0, lep.p1);
    }
}

void generate_position_rectangle(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    //const EmitterGeometry::LineEmitterGeometry& lep = ep.lep;
}

void generate_position_cube(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    //const EmitterGeometry::LineEmitterGeometry& lep = ep.lep;
}

void generate_position_circle(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    //const EmitterGeometry::LineEmitterGeometry& lep = ep.lep;
}

void generate_position_sphere(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    //const EmitterGeometry::LineEmitterGeometry& lep = ep.lep;
}

static const generator_t gs_generators[] = {
    &generate_position_line,             // Line
    &generate_position_rectangle,        // Rectangle
    &generate_position_cube,             // Cube
    &generate_position_circle,           // Circle
    &generate_position_sphere            // Sphere
};

void generate_position(const EmitterGeometry& ep, vec3* pos, size_t count)
{
    assert(ep.type_ >= 0 && ep.type_ < sizeof(gs_generators)/sizeof(gs_generators[0]));
    gs_generators[ep.type_](ep, pos, count);

        //float theta = random(s_dir.x, e_dir.x);
        //float phi = random(s_dir.y, e_dir.y);
        //dir[i] = spherical_to_cartesian(theta, phi);
}

vec3 spherical_to_cartesian(float theta, float phi)
{
    vec3 rv;
    float sin_theta, cos_theta;
    float sin_phi, cos_phi;
    sincosf(theta, &sin_theta, &cos_theta);
    sincosf(phi, &sin_phi, &cos_phi);

    // Y - is up
    rv.x = sin_theta * cos_phi;
    rv.z = sin_theta * sin_phi;
    rv.y = cos_theta;

    return rv;
}

void generate_dir(const vec2& s_dir, const vec2& e_dir, vec3* dir, size_t count)
{
    for(uint32_t i = 0; i < count; ++i)
    {
        float theta = random(s_dir.x, e_dir.x);
        float phi = random(s_dir.y, e_dir.y);
        dir[i] = spherical_to_cartesian(theta, phi);
    }
}

void generate_velocity(const float s, const float e, float * vel, size_t count)
{
    for(uint32_t i = 0; i < count; ++i)
    {
        vel[i] = random(s, e);
    }
}

void generate_accel(const float s, const float e, float* acc, size_t count)
{
    for(uint32_t i = 0; i < count; ++i)
    {
        acc[i] = random(s, e);
    }
}

void generate_lifetime(const float s, const float e, vec2* lifetime, size_t count)
{
    for(uint32_t i = 0; i < count; ++i)
    {
        lifetime[i].x = 0.0f;
        lifetime[i].y = random(s, e);
    }
}


} //namespace 

class StandardEmitter: public ParticleEmitterInterface {

    EmitterGeometry eg_;
    // generate parameters
    float rate_;  // part/sec
    vec2 angle_from_, angle_to_;
    float velocity_from_, velocity_to_;
    float accel_from_, accel_to_;
    float lifetime_from_, lifetime_to_;

    // todo use transform component for this
    vec3 position_;
    vec3 direction_;

    size_t size_;
    size_t capacity_;
    vec3* pos_;
    vec3* dir_;
    float* v_;
    float* acc_;
    vec2* lifetime_;

    static const uint32_t NUM_BUFFERS = 3;
    static const uint32_t MAX_VB_SIZE = 102400;
    uint32_t cur_vb_;
    float num_particles_to_generate_;
    HGOSBUFFER vb_[NUM_BUFFERS];
    HGOSBUFFER ib_;
    //size_t vb_size_;
    //size_t ib_size_;

    void reallocate(size_t new_capacity)
    {
        vec3 * pos, *dir;
        float* v, *acc;
        vec2 * lifetime;
        if(new_capacity)
        {
            pos = new vec3[new_capacity];
            dir = new vec3[new_capacity];
            v = new float[new_capacity];
            acc = new float[new_capacity];
            lifetime = new vec2[new_capacity];
        }

        capacity_ = new_capacity;
        size_ = capacity_ <= size_ ? capacity_ : size_;

        if(new_capacity)
        {
            memcpy(pos, pos_, size_ * sizeof(vec3));
            memcpy(dir, dir_, size_ * sizeof(vec3));
            memcpy(v, v_, size_ * sizeof(float));
            memcpy(acc, acc_, size_ * sizeof(float));
            memcpy(lifetime, lifetime_, size_ * sizeof(vec2));
        }

        delete[] pos_;
        delete[] dir_;
        delete[] v_;
        delete[] acc_;
        delete[] lifetime_;

        if(new_capacity)
        {
            pos_ = pos;
            dir_ = dir;
            v_ = v;
            acc_ = acc;
            lifetime_ = lifetime;
        }
    }

    void assign(uint32_t i_to, uint32_t i_from)
    {
        pos_[i_to] = pos_[i_from];
        dir_[i_to] = dir_[i_from];
        v_[i_to] = v_[i_from];
        acc_[i_to] = acc_[i_from];
        lifetime_[i_to] = lifetime_[i_from];
    }

public:

    static HGOSVERTEXDECLARATION get_vdecl() {
	   static auto vdecl = gos_CreateVertexDeclaration(particle_vdecl, sizeof(particle_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
       return vdecl;
    }

    static HGOSRENDERMATERIAL get_material() {
        static auto mat = gos_getRenderMaterial("particle");
        return mat;
    }

    static HGOSBUFFER get_quad_ib() {
        uint16_t ib_data[] = { 0, 2, 3, 0, 3, 1} ;
        static auto ib = gos_CreateBuffer(
            gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
            sizeof(ParticleVDecl), sizeof(ib_data) / sizeof(ib_data[0]),
            ib_data);
        return ib;
    }

#if 0
    static HGOSBUFFER get_quad_vb() {
        ParticleVDecl vb_data[] = {{vec2(0.0f, 0.0f)},
                                   {vec2(0.0f, 1.0f)},
                                   {vec2(1.0f, 0.0f)},
                                   {vec2(1.0f, 1.0f)}};
        static auto vb = gos_CreateBuffer(
            gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
            sizeof(ParticleVDecl), sizeof(vb_data) / sizeof(vb_data[0]),
            vb_data);
        return vb;
    }
#else
    static HGOSBUFFER get_quad_vb(){
        ParticleVDecl vb_data[] = {{vec2(0.0f, 0.0f)},  // 0
                                   {vec2(1.0f, 0.0f)},  // 2
                                   {vec2(1.0f, 1.0f)},  // 3
                                   {vec2(0.0f, 0.0f)},  // 0
                                   {vec2(1.0f, 1.0f)},  // 3
                                   {vec2(0.0f, 1.0f)}}; // 1

        static auto vb =
            gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                             gosBUFFER_USAGE::STATIC_DRAW,
                             sizeof(ParticleVDecl),
                             sizeof(vb_data) / sizeof(vb_data[0]), vb_data);
        return vb;
    }
#endif

    size_t GetCount() const override { return size_; }

    void Create() {

        eg_.type_ = Type::Line;
        eg_.lep_.p0 = vec3(0.0f);
        eg_.lep_.p1 = vec3(1.0f);

        position_ = vec3(0.0f);
        direction_ = vec3(0.0f, 1.0f, 0.0f);

        rate_ = 30.0f;
        angle_from_ = vec2(0.0f, 0.0f);
        angle_to_ = vec2((1.0f / 3.0f) * 3.1415f, 2.0f * 3.1415f);
        velocity_from_ = 1.0f;
        velocity_to_ = 4.0f;
        accel_from_ = 0.0f;
        accel_to_ = 2.5f;
        lifetime_from_ = 4.0f;
        lifetime_to_ = 7.0f;

        size_ = 0;
        capacity_ = 100;
        pos_ = new vec3[capacity_];
        dir_ = new vec3[capacity_];
        v_ = new float[capacity_];
        acc_ = new float[capacity_];
        lifetime_ = new vec2[capacity_]; // x: current, y: max

        cur_vb_ = 0;
        for(uint32_t i=0;i<NUM_BUFFERS;++i)
            vb_[i] = 0;
        ib_ = 0;
        num_particles_to_generate_ = 0;
    }

    void Destroy()
    {
        reallocate(0);
    }

    HGOSRENDERMATERIAL GetMaterial() const override { return get_material(); }

    void Update(const float dt) override {

        float clamped_dt_sec = clamp(dt / 1000.0f, 0.0f, 0.1f);

        num_particles_to_generate_ += rate_ * clamped_dt_sec;
        float num2gen = floor(num_particles_to_generate_);
        num_particles_to_generate_ -= num2gen;

        uint32_t new_count = (uint32_t)num2gen;

        uint32_t old_size = size_;
        if(size_ + new_count > capacity_)
        {
            assert(size_ + new_count <= MAX_VB_SIZE/sizeof(ParticleInstVDecl));
            // after this size_ will be updated!
            reallocate(size_ + new_count);
        }

        generate_position(eg_, pos_ + old_size, new_count);
        generate_dir(angle_from_, angle_to_, dir_ + old_size, new_count);
        generate_velocity(velocity_from_, velocity_to_, v_ + old_size, new_count);
        generate_accel(accel_from_, accel_to_, acc_+ old_size, new_count);
        generate_lifetime(lifetime_from_, lifetime_to_, lifetime_ + old_size, new_count);

        size_ += new_count;
        for(uint32_t i = 0; i < size_; ++i)
        {
            lifetime_[i].x += clamped_dt_sec;
            if(lifetime_[i].x > lifetime_[i].y) // if  > max
            {
                assign(i, size_ - 1);
                size_--;
            }

            // update direction
            // ...

            v_[i] = v_[i] + acc_[i]*clamped_dt_sec;
            pos_[i] = pos_[i] + dir_[i]*v_[i]*clamped_dt_sec;
        }
    }

    void FillVertexBuffer(ParticleInstVDecl* vb, size_t& out_count)
    {
        for(uint32_t i = 0; i < size_; ++i)
        {
            vb[i].pos_ = pos_[i];
            vb[i].lifetime_ = lifetime_[i].x / lifetime_[i].y;
        }
        out_count = size_;
    }

    virtual void AddRenderPacket(struct RenderFrameContext* rfc) override
    {
        if(!size_)
            return;

        RenderPacket* rp = rfc->rl_->AddPacket();
    
        UpdateVertexData(rfc);

        rp->mesh_.ib_ = 0;
        rp->mesh_.inst_vb_ = vb_[cur_vb_];
        rp->mesh_.vb_ = get_quad_vb();
        rp->mesh_.tex_id_ = 0;
        rp->mesh_.vdecl_ = get_vdecl();
        rp->mesh_.num_instances = size_;
        rp->m_ = mat4::identity();
        rp->is_render_to_shadow = 0;
        rp->is_transparent_pass = 1;
    }

    void UpdateVertexData(struct RenderFrameContext* rfc)
    {
        if(!size_)
            return;
            
        cur_vb_ = (cur_vb_ + 1) % NUM_BUFFERS;
        HGOSBUFFER vb = vb_[cur_vb_];
        uint32_t size = size_;
        assert(vb);

        ParticleInstVDecl* pdata = new ParticleInstVDecl[size_];
        for (uint32_t i = 0; i < size; ++i) {
            pdata[i].pos_ = pos_[i];
            pdata[i].lifetime_ = lifetime_[i].x / lifetime_[i].y;
        }

        ScheduleRenderCommand(rfc, [vb, size, pdata]() {

            ParticleInstVDecl *particles = (ParticleInstVDecl *)gos_MapBuffer(
            vb, 0, size,
            gosBUFFER_ACCESS::WRITE/* | gosBUFFER_ACCESS::NO_SYNC |
                gosBUFFER_ACCESS::COHERENT*/);

            memcpy(particles, pdata, size * sizeof(ParticleInstVDecl));

            gos_UnmapBuffer(vb);
            delete[] pdata;
        });
    }

    void InitRenderResources() override
    {
        for(uint32_t i=0; i < NUM_BUFFERS; ++i)
        {
            vb_[i] = gos_CreateBuffer(
                gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
                sizeof(ParticleInstVDecl), MAX_VB_SIZE, nullptr);
        }
    }

    void DestroyRenderResources() override
    {
        for(uint32_t i=0; i < NUM_BUFFERS; ++i)
            gos_DestroyBuffer(vb_[i]);
    }
};

void ParticleSystem::Update(const float dt)
{
    for(auto e: emitters_)
        e->Update(dt);
}

void ParticleSystem::Render(struct RenderFrameContext *rfc)
{
    for(auto e: emitters_)
        e->AddRenderPacket(rfc);
}

void ParticleSystem::InitRenderResources()
{
    for(auto e: emitters_)
        e->InitRenderResources();
}
void ParticleSystem::DestroyRenderResources()
{
    for(auto e: emitters_)
        e->DestroyRenderResources();
}

void ParticleSystemManager::Update(const float dt)
{
    for(auto ps: ps_list_)
        ps->Update(dt);
}

void ParticleSystemManager::Render(struct RenderFrameContext *rfc)
{
    for(auto ps: ps_list_)
        ps->Render(rfc);
}

void ParticleSystemManager::DestroyRenderResources()
{
    gos_DestroyBuffer(StandardEmitter::get_quad_ib());
    gos_DestroyVertexDeclaration(StandardEmitter::get_vdecl());
}




////////////////////////////////////////////////////////////////////////////////

ParticleEmitterInterface* CreateStandardEmitter()
{
    StandardEmitter* se = new StandardEmitter;
    se->Create();
    return se;
}


////////////////////////////////////////////////////////////////////////////////

class ParticleRenderer {

	mat4 view_;
	mat4 proj_;

public:

	void setup(const mat4& view, const mat4& proj) {
		view_ = view;
        proj_ = proj;
	}

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("particle");

        gos_SetRenderState(gos_State_Texture, ro.tex_id_);
        gos_SetRenderState(gos_State_Filter, gos_FilterBiLinear);

        mat4 vp = proj_ * view_;
        mat4 wvp = vp * rp.m_;
        mat4 wv = view_ * rp.m_;
        float has_texture[] = { ro.tex_id_ ? 1.0f : 0.0f, 
                                0.0f,
                                0.0f, 
                                0.0f };

		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		gos_SetRenderMaterialParameterMat4(mat, "wv_", (const float*)wv);
		gos_SetRenderMaterialParameterMat4(mat, "proj_", (const float*)proj_);
		gos_SetRenderMaterialParameterFloat4(mat, "has_texture", has_texture);

		gos_ApplyRenderMaterial(mat);

        if (ro.ib_) {
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);
        } else if(ro.inst_vb_) {
    		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_);
        } else {
    		gos_RenderArray(ro.vb_, ro.vdecl_);
        }
    }
};

void RenderParticles(const RenderPacketList_t& rpl, const mat4& view, const mat4& proj)
{
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    ParticleRenderer pr;
    pr.setup(view, proj);

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaInvAlpha);
    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        if(rp.is_transparent_pass)
            pr.render(rp);
    }
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
}

