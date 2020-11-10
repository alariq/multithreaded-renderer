#include "sph.h"
#include <cmath>
#include "obj_model.h"
#include "res_man.h"

// solver parameters
const static vec2 G(0.f, 120 * -9.8f); // external (gravitational) forces
const static float REST_DENS = 1000.f;	 // rest density
const static float GAS_CONST = 2000.f;	 // const for equation of state
const static float H = 0.2f;//16.f;			 // kernel radius
const static float HSQ = H * H;			 // radius^2 for optimization
const static float MASS = .65f;//65.f;			 // assume all particles have the same mass
const static float VISC = 250.f;		 // viscosity constant
const static float DT = 0.0008f;		 // integration timestep

// smoothing kernels defined in M�ller and their gradients
const static float POLY6 = 315.f / (65.f * M_PI * pow(H, 9.f));
const static float SPIKY_GRAD = -45.f / (M_PI * pow(H, 6.f));
const static float VISC_LAP = 45.f / (M_PI * pow(H, 6.f));

// simulation parameters
const static float EPS = H; // boundary epsilon
const static float BOUND_DAMPING = -0.5f;

void ComputeDensityPressure(SPHParticle2D *particles, int count) {
	for (int i = 0; i < count; ++i) {
		SPHParticle2D &pi = particles[i];
		pi.density = 0.f;
		for (int j = 0; j < count; ++j) {

			SPHParticle2D &pj = particles[j];
			vec2 rij = pj.pos - pi.pos;
			float r2 = lengthSqr(rij);

			if (r2 < HSQ) {
				// this computation is symmetric
				pi.density += MASS * POLY6 * pow(HSQ - r2, 3.f);
			}
		}
		pi.pressure = GAS_CONST * (pi.density - REST_DENS);
	}
}

void ComputeForces(SPHParticle2D *particles, int count)
{
	for (int i = 0; i < count; ++i) {
	    SPHParticle2D &pi = particles[i];

        vec2 fpress(0.f, 0.f);
        vec2 fvisc(0.f, 0.f);

		for (int j = 0; j < count; ++j) {
			SPHParticle2D &pj = particles[j];
        
            if(i == j)
                continue;

            vec2 rij = pj.pos - pi.pos;
            float r = length(rij);

            if(r < H)
            {
                // compute pressure force contribution
				fpress += -normalize(rij) * MASS * (pi.pressure + pj.pressure) /
						  (2.f * pj.density) * SPIKY_GRAD * pow(H - r, 2.f);
                // compute viscosity force contribution
				fvisc += VISC * MASS * (pj.vel - pi.vel) / pj.density * VISC_LAP * (H - r);
            }
        }
        vec2 fgrav = G * pi.density;
        pi.force = fpress + fvisc + fgrav;
    }
}

void Integrate(SPHParticle2D* particles, int count, vec2 view_dim)
{
    for (int i = 0; i < count; ++i) {
        SPHParticle2D& p = particles[i];

        // forward Euler integration
        p.vel += DT * p.force / p.density;
        p.pos += DT * p.vel;

        // enforce boundary conditions
        if (p.pos.x - EPS < 0.0f)
        {
            p.vel.x *= BOUND_DAMPING;
            p.pos.x = EPS;
        }
        if (p.pos.x + EPS > view_dim.x)
        {
            p.vel.x *= BOUND_DAMPING;
            p.pos.x = view_dim.x - EPS;
        }
        if (p.pos.y - EPS < 0.0f)
        {
            p.vel.y *= BOUND_DAMPING;
            p.pos.y = EPS;
        }
        if (p.pos.y + EPS > view_dim.y)
        {
            p.vel.y *= BOUND_DAMPING;
            p.pos.y = view_dim.y - EPS;
        }
    }
}
 
void sph_init_renderer() {
    gos_AddRenderMaterial("deferred_sph");
}

void sph_update(SPHParticle2D *particles, int count, vec2 view_dim) {
	ComputeDensityPressure(particles, count);
	ComputeForces(particles, count);
	Integrate(particles, count, view_dim);
}

HGOSVERTEXDECLARATION get_sph_vdecl() {

	static gosVERTEX_FORMAT_RECORD sph_vdecl[] = {
		// SVD
		{0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
		{1, 2, false, sizeof(SVD), offsetof(SVD, uv), gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
		{2, 3, false, sizeof(SVD), offsetof(SVD, normal), gosVERTEX_ATTRIB_TYPE::FLOAT, 0},

		// instance data: stream 1
		{3, 2, false, sizeof(SPHInstVDecl), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
		{4, 2, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,vel), gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
		{5, 2, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,force), gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
		{6, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,density), gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
		{7, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,pressure), gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
	};

    static auto vdecl = gos_CreateVertexDeclaration(
        sph_vdecl, sizeof(sph_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}

SPHSceneObject* SPHSceneObject::Create(const vec2& view_dim, int num_particles, const vec3& pos) {
    SPHSceneObject* o = new SPHSceneObject();
    o->particles_ = new SPHParticle2D[num_particles];
    o->num_particles_ = num_particles;
    o->view_dim_ = view_dim;
    auto tr = o->AddComponent<TransformComponent>();
    tr->SetPosition(pos);
    o->transform_ = tr;
    return o;
}

SPHSceneObject::~SPHSceneObject() {
    DeinitRenderResources();
    delete particles_;
}

extern int RendererGetNumBufferedFrames();
void SPHSceneObject::InitRenderResources() {
	sphere_mesh_ = res_man_load_mesh("sphere");

    int num_buffers = RendererGetNumBufferedFrames() + 1;
    inst_vb_.resize(num_buffers);
	for (int32_t i = 0; i < num_buffers; ++i) {
		inst_vb_[i] =
			gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
							 sizeof(SPHInstVDecl), num_particles_, nullptr);
	}

    vdecl_ = get_sph_vdecl();
    mat_ = gos_getRenderMaterial("deferred_sph");
}

void SPHSceneObject::DeinitRenderResources() {
    delete sphere_mesh_;
    sphere_mesh_ = nullptr;
    for (auto& buf : inst_vb_) {
        gos_DestroyBuffer(buf);
    }
    inst_vb_.clear();
    gos_DestroyVertexDeclaration(vdecl_);
    vdecl_ = nullptr;
}

void SPHSceneObject::Update(float /*dt*/) {

    sph_update(particles_, num_particles_, view_dim_);

   
}

void SPHSceneObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	// update instancing buffer
	cur_inst_vb_ = (cur_inst_vb_ + 1) % ((int)inst_vb_.size());

	int num_particles = num_particles_;
	HGOSBUFFER inst_vb = inst_vb_[cur_inst_vb_];
	SPHParticle2D *particles = particles_;
	ScheduleRenderCommand(rfc, [num_particles, inst_vb, particles]() {
		const size_t bufsize = num_particles * sizeof(SPHParticle2D);
		// TODO: think about typed buffer wrapper
		SPHParticle2D *part_data =
			(SPHParticle2D *)gos_MapBuffer(inst_vb, 0, bufsize, gosBUFFER_ACCESS::WRITE);
		memcpy(part_data, particles, bufsize);
		gos_UnmapBuffer(inst_vb);
	});

    class RenderList* rl = rfc->rl_;
	RenderPacket *rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
	rp->is_selection_pass = 1;
    rp->m_ = transform_->GetTransform(); //mat4::scale(vec3(radius_));
	rp->mesh_ = *sphere_mesh_;
	rp->mesh_.mat_ = mat_;
	rp->mesh_.inst_vb_ = inst_vb_[cur_inst_vb_];
    rp->mesh_.vdecl_ = vdecl_;
    rp->mesh_.num_instances = num_particles_;
}

void initialize_particle_positions(SPHSceneObject* o) {
    SPHParticle2D*  particles = o->GetParticles();
    const int count = o->GetParticlesCount();
    const float radius = o->GetRadius();
    vec2 offset = vec2(o->GetBounds().x*0.2f, o->GetBounds().y*0.1f);
    int row_size = 20;
    int column_size = (count + row_size - 1) / row_size; 
    for(int y= 0; y<row_size; ++y) {
        for(int x= 0; x<column_size; ++x) {
            int idx = y*column_size + x;
            if(idx >= count)
                break;
            SPHParticle2D& p = particles[idx];
            float jitter = random(-0.02f, 0.02f);
            p.pos = offset + vec2(x*2.1f*radius + jitter, y*2.1f*radius);
            p.pressure = 0;
            p.density = 0;
            p.force = vec2(0,0);
            p.vel = vec2(0,0);
        }
    }
}
