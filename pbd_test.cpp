#include "pbd_test.h"
#include "pbd/pbd_particles.h"
#include "res_man.h"
#include "utils/vec.h"
#include "utils/camera_utils.h" // screen2world_vec
#include "utils/kernels.h"
#include "utils/math_utils.h"

void initialize_particle_positions(struct PBDUnifiedSimulation* sim, const vec2& dim, int count, float density0);
void initialize_fluid_particle_positions(struct PBDUnifiedSimulation* sim,
										 const vec2& dim, const vec2& offset,
										 int row_size, int count, int fluid_model_idx);

void pbd_unified_sim_debug_draw(const struct PBDUnifiedSimulation* sim, class RenderList* rl);

void scene_stacking_particles_and_box_above(PBDUnifiedSimulation* sim) {

    pbd_unified_sim_add_box_rigid_body(sim, 5, 4, vec2(1.0, 4), 0, 1000);
    vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
    initialize_particle_positions(sim, world_size, 10, 1000);
}

void scene_friction_test(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;
    const float radius = pbd_unified_sim_get_particle_radius(sim);
	vec2 offset = vec2(1.2f, 1.0f * radius);
    const float column_size = 4;
    const float row_size = 10;

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);
    
	for (int y = 0; y < row_size; ++y) {
		for (int x = 0; x < column_size; ++x) {

			float jitter = 0; // random(-0.01f, 0.01f);
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			pbd_unified_sim_add_particle(sim, pos, density0);
		}
	}
}

void scene_rb_friction_test(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);

    //vec2 rb_pos = vec2(0.2f, 3.2f);
    //int rb_idx = pbd_unified_sim_add_box_rigid_body(sim, 5, 4, rb_pos, 1*30.0f* 3.1415f/180.0f, density0);
    //pbd_unified_sim_rb_add_velocity(sim, rb_idx, vec2(3, 0));
    
    int rb2_idx = pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(2,0.3f), 0*30.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb2_idx, vec2(10, 0));

    int rb3_idx = pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(2,2.0f), 0*30.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb3_idx, vec2(5, 0));


    pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(0.6f, 0.7f), -65.0f* 3.1415f/180.0f, density0);
}

void scene_rb_friction_test2(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;
    const float radius = pbd_unified_sim_get_particle_radius(sim);

	//pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);

    int rb_dim_x = 4;
    int rb_dim_y = 4;
    vec2 rb_size = 2 * vec2(rb_dim_x*radius, rb_dim_y*radius);

	vec2 offset = vec2(0.0f, 0.0f) + 1.0f * vec2(rb_size.x, rb_size.y);
    const float column_size = 4;
    const float row_size = 4;

	for (int y = 0; y < row_size; ++y) {
		for (int x = 0; x < column_size; ++x) {

			float jitter = random(-0.01f, 0.01f);
			vec2 pos = offset + vec2(x * 1.5f*rb_size.x + jitter, y * 1.5f * rb_size.y);
            float rot = random(0.0f, 1.0f);

            pbd_unified_sim_add_box_rigid_body(sim, rb_dim_x, rb_dim_y, pos, rot * 2* 3.1415f, density0);
		}
	}
}

void scene_particle_box_collision_test(PBDUnifiedSimulation* sim) {
    const float density0 = 1000;
    pbd_unified_sim_add_box_rigid_body(sim, 5, 4, vec2(4.0, 0.4f), 0, density0);

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 0.5f), vec2(7,3.0f), density0);
}

void scene_complex(PBDUnifiedSimulation* sim) {
    vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
    initialize_particle_positions(sim, world_size, 10, 1000);

    vec2 rb_pos = vec2(0.2f, 3.2f);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos, 0*45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-0.6f), 0*-45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-2*0.6f),0* 45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-3*0.6f),0* -45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-4*0.6f),0* 45.0f* 3.1415f/180.0f, 1000);

    for(int i=0; i< 2; i++) {
        vec2 pos = vec2(1.6f, 4.5f - i*1.4f);
        float r = 0;//(i%2) ? -15.0f* 3.1415f/180.0f : 15.0f* 3.1415f/180.0f;
        pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
    }
}

void scene_fluid_simple(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    float desired_density0 = 100;

    int fm_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0);
	vec2 offset = vec2(0.5f, 1.0f * radius);
	int row_size = 16;
    initialize_fluid_particle_positions(sim, world_size, offset, row_size, 200, fm_idx);
    vec2 rb_pos = vec2(world_size.x*0.5f, world_size.y - 12*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos, 0*45.0f* 3.1415f/180.0f, 80);

#if 1
    vec2 rb_pos2 = vec2(world_size.x*0.5f + 0*(3*2*radius + 0.5f), world_size.y - 3*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos2, 0*45.0f* 3.1415f/180.0f, 250);
    
    vec2 rb_pos3 = vec2(world_size.x*0.1f + 3*2*radius + 0.5f, world_size.y - 3*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 1, 1, rb_pos3, 0*45.0f* 3.1415f/180.0f, 30);

#endif
#if 0

    for(int i=0; i< 2; i++) {
        vec2 pos = vec2(1.6f, 4.5f - i*1.4f);
        float r = 0;//(i%2) ? -15.0f* 3.1415f/180.0f : 15.0f* 3.1415f/180.0f;
        if(i==0)
            pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
        else
            pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
    }
#endif
}


PBDTestObject* PBDTestObject::Create() {

    PBDTestObject* o = new PBDTestObject;
    o->sim_dim_ = vec2(5,5);
    o->sim_origin_ = vec2(0,0);
    o->sim_ = pbd_unified_sim_create(o->sim_dim_);

    //scene_stacking_particles_and_box_above(o->sim_);
    //scene_complex(o->sim_);
    //scene_particle_box_collision_test(o->sim_);
    //scene_friction_test(o->sim_);
    //scene_rb_friction_test(o->sim_);
    //scene_rb_friction_test2(o->sim_);
    scene_fluid_simple(o->sim_);

	return o;
}

PBDTestObject::~PBDTestObject() {
   pbd_unified_sim_destroy(sim_); 
}

struct PBDParticleVDecl {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
    uint32_t flags;
};

static HGOSVERTEXDECLARATION get_pbd_vdecl() {

	static gosVERTEX_FORMAT_RECORD pbd_vdecl[] = {
		// SVD
		{0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{1, 2, false, sizeof(SVD), offsetof(SVD, uv), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{2, 3, false, sizeof(SVD), offsetof(SVD, normal), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},

		// instance data: stream 1
		{3, 2, false, sizeof(PBDParticleVDecl), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{4, 2, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,vel), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{5, 2, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,force), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{6, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,density), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{7, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,pressure), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{8, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,flags), gosVERTEX_ATTRIB_TYPE::kUNSIGNED_INT, 1},
	};

    static auto vdecl = gos_CreateVertexDeclaration(
        pbd_vdecl, sizeof(pbd_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}


extern int RendererGetNumBufferedFrames();
void PBDTestObject::InitRenderResources() {
	sphere_mesh_ = res_man_load_mesh("sphere");

	gos_AddRenderMaterial("deferred_pbd_particle");

    int num_buffers = RendererGetNumBufferedFrames() + 1;
    inst_vb_.resize(num_buffers);
	for (int32_t i = 0; i < num_buffers; ++i) {
		inst_vb_[i] =
			gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
							 sizeof(PBDParticleVDecl), 1000, nullptr);
	}

    vdecl_ = get_pbd_vdecl();
	mat_ = gos_getRenderMaterial("deferred_pbd_particle");

    b_initalized_rendering_resources = true;

}

void PBDTestObject::DeinitRenderResources() {
    b_initalized_rendering_resources = false;

    delete sphere_mesh_;
    sphere_mesh_ = nullptr;
    for (auto& buf : inst_vb_) {
        gos_DestroyBuffer(buf);
    }
    inst_vb_.clear();
    gos_DestroyVertexDeclaration(vdecl_);
    vdecl_ = nullptr;
}

void PBDTestObject::Update(float dt) {

    if(gos_GetKeyStatus(KEY_A) == KEY_PRESSED) {
        // add particle
    }

    pbd_unified_timestep(sim_, dt);
}

extern void render_quad(uint32_t tex_id, const vec4& scale_offset, HGOSRENDERMATERIAL pmat);
void update_closest_dist_debug_line(struct RenderFrameContext *rfc);

void PBDTestObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	if (!b_initalized_rendering_resources) return;

    pbd_unified_sim_debug_draw(sim_, rfc->rl_);

    //update_closest_dist_debug_line(rfc);

	// update instancing buffer
	cur_inst_vb_ = (cur_inst_vb_ + 1) % ((int)inst_vb_.size());

	const int num_particles = pbd_unified_sim_get_particle_count(sim_);
	HGOSBUFFER inst_vb = inst_vb_[cur_inst_vb_];
	const PBDParticle* particles = pbd_unified_sim_get_particles(sim_);

	if (num_particles) {
	ScheduleRenderCommand(rfc, [num_particles, inst_vb, particles]() {
		const size_t bufsize = num_particles * sizeof(PBDParticleVDecl);
		// TODO: think about typed buffer wrapper
			int inst_buf_num_part =
				(int)(gos_GetBufferSizeBytes(inst_vb) / sizeof(PBDParticleVDecl));
			if (inst_buf_num_part < num_particles) {
				gos_ResizeBuffer(inst_vb, (uint32_t)(num_particles * 1.5f));
        }

			PBDParticleVDecl* part_data = (PBDParticleVDecl*)gos_MapBuffer(
				inst_vb, 0, bufsize, gosBUFFER_ACCESS::WRITE);
		for (int i = 0; i < num_particles; ++i) {
			PBDParticleVDecl& p = part_data[i];

			p.pos = particles[i].x;
			p.vel = particles[i].v;
			p.force = vec2(0);
			p.density = 1;
			p.pressure = 1;
			p.flags = particles[i].flags;
		}

		gos_UnmapBuffer(inst_vb);

#ifdef DEBUG_DRAW_PARTICLE_OUTLINE
			for (int i = 0; i < num_particles; ++i) {
            vec3 p = vec3(particles[i].pos.x, particles[i].pos.y, 0);
				// ceneter
				gos_AddPoints(&p, 1, vec4(1, 1, 1, 1), 4);
				for (int j = 0; j < 36; ++j) {
					vec3 dp = p + 0.1 * vec3(sin((float)j * 10 * M_PI / 180.0f),
											 cos((float)j * 10 * M_PI / 180.0f), 0.0f);
					gos_AddPoints(&dp, 1, vec4(1, 1, 1, 1), 4);
            }
        }
#endif
	});
	}

	class RenderList* rl = rfc->rl_;
	RenderPacket *rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
	rp->is_selection_pass = 1;
    rp->m_ = mat4::identity();//transform_->GetTransform(); //mat4::scale(vec3(radius_));
	rp->mesh_ = *sphere_mesh_;
	rp->mesh_.mat_ = mat_;
	rp->mesh_.inst_vb_ = inst_vb_[cur_inst_vb_];
    rp->mesh_.vdecl_ = vdecl_;
    rp->mesh_.num_instances = num_particles;
}

void initialize_particle_positions(struct PBDUnifiedSimulation* sim, const vec2& dim, int count,
								   float density0) {

	const float radius = pbd_unified_sim_get_particle_radius(sim);
	vec2 offset = vec2(1.2f, 1.0f * radius+ 1);
	int row_size = 1;//(int)(sqrtf((float)count) + 1.5f);
	int column_size = (count + row_size - 1) / row_size;
	for (int y = 0; y < column_size; ++y) {
		for (int x = 0; x < row_size; ++x) {

			int idx = y * row_size + x;
			if (idx >= count) break;

			float jitter = 0; // random(-0.01f, 0.01f);
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			pbd_unified_sim_add_particle(sim, pos, density0);
		}
	}
}

void initialize_fluid_particle_positions(struct PBDUnifiedSimulation* sim,
										 const vec2& dim, const vec2& offset, int row_size, int count,
										 int fluid_model_idx) {

	const float radius = pbd_unified_sim_get_particle_radius(sim);
	int column_size = (count + row_size - 1) / row_size;
	for (int y = 0; y < column_size; ++y) {
		for (int x = 0; x < row_size; ++x) {

			int idx = y * row_size + x;
			if (idx >= count) break;

			float jitter = 0; // random(-0.01f, 0.01f);
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			pbd_unified_sim_add_fluid_particle(sim, pos, fluid_model_idx);
		}
	}
}

void pbd_unified_sim_debug_draw(const struct PBDUnifiedSimulation* sim, RenderList* rl) {

    const PBDRigidBodyParticleData* data = pbd_unified_sim_get_rb_particle_data(sim);
    const PBDRigidBody* rb = pbd_unified_sim_get_rigid_bodies(sim);

    const int count = pbd_unified_sim_get_particle_count(sim);
    const PBDParticle* particles = pbd_unified_sim_get_particles(sim);

    const float z = -0.1f;

	for (int i = 0; i < count; ++i) {
		const PBDParticle& p = particles[i];
		if (p.flags & PBDParticleFlags::kRigidBody) {
			const PBDRigidBodyParticleData& data_i = data[p.rb_data_idx];
			const PBDRigidBody& rb_i = rb[p.phase];
			vec2 grad = rotate2(rb_i.angle) * data_i.sdf_grad;
			rl->addDebugLine(vec3(p.x, 0), vec3(p.x, 0) + 0.15f * vec3(grad, 0),
							 vec4(1, 1, 0.5f, 1));
		}
	}

	vec2 world_bounds = pbd_unified_sim_get_world_bounds(sim);
	rl->addDebugLine(vec3(0, 0, z), vec3(world_bounds.x, 0, z), vec4(1));
	rl->addDebugLine(vec3(world_bounds.x, 0, z), vec3(world_bounds, z), vec4(1));
	rl->addDebugLine(vec3(world_bounds, z), vec3(0, world_bounds.y, z), vec4(1));
	rl->addDebugLine(vec3(0, world_bounds.y, z), vec3(0, 0, z), vec4(1));

	//for(auto rb: sim->rigid_bodies_) {
    //}
}
