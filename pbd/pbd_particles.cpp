// Unified Particle Physics for Real-Time Applications

#include "pbd_particles.h"
#include "engine/utils/kernels.h"

#include <cmath>
#include <vector>
#include <cassert>

#define MAKE_COLLISIONS_GREAT_AGAIN 1
// this prefers normal straight along collision line, which works better just because
// corner particles in rigid bodies have gradients which are not always "correct"
// Maybe makes sense to query gradient of J RB from position of I particle

// investigate why particles go nuts and enable after that, because enabling this, will
// hide the problem
#define ENABLE_CFL 1

struct Pair {
	int start;
	int num;
};

struct NeighbourData {
	std::vector<int> neigh_idx_;
	std::vector<Pair> neighbour_info_;
};

struct ShapeMatchingConstraint {
    int rb_index;
};

struct RigidBodyCollisionConstraint {
    int idx0, idx1;
    vec2 x_ij;
};

struct ParticleRigidBodyCollisionConstraint {
    // idx0 - rigid body particle
    // idx1 - ordinary particle
    int idx0, idx1;
    // xi - xj, xi <-------- xj
    vec2 x_ij;
};

struct SolidParticlesCollisionConstraint {
    int idx0, idx1;
    // xi - xj, xi <-------- xj
    vec2 x_ij;
};

struct BoxBoundaryConstraint {
    //bool b_invert;
    vec2 p_min;
    vec2 p_max;
    float mu_s, mu_k;
};

void findNeighboursNaiive(const vec2* x, const int size, float radius,
						  PBDParticle* particles, NeighbourData* nd) {
	nd->neigh_idx_.resize(0);

	int offset = 0;
	const float r_sq = radius * radius;

	for (int i = 0; i < size; i++) {
		vec2 x_i = x[i];
		int start = offset;
		for (int j = 0; j < size; j++) {

			if (i == j) {
                continue;
            }

			// skip particles of same rigid body, maybe leave "j" in neighbours but just use
			// this condition when checking constraints?
			bool b_is_same_rb = particles[i].phase == particles[j].phase &&
							  (particles[i].flags & PBDParticleFlags::kRigidBody) &&
                              // this condition is redundant
							  (particles[j].flags & PBDParticleFlags::kRigidBody);
			bool b_phase_cond = !b_is_same_rb;
			if (b_phase_cond && lengthSqr(x_i - x[j]) < r_sq) {
				nd->neigh_idx_.push_back(j);
				offset++;
			}
		}
		nd->neighbour_info_[i].start = start;
		nd->neighbour_info_[i].num = offset - start;
	}
}

struct PBDUnifiedSimulation {

    static constexpr const float particle_r_ = 0.1f;
    static constexpr const float support_r_ = particle_r_ * 4.0f;

    vec2 world_size_ = vec2(5,5);

	std::vector<SolidParticlesCollisionConstraint> solid_collision_c_;
	std::vector<ShapeMatchingConstraint> shape_matching_c_;
	std::vector<RigidBodyCollisionConstraint> rb_collision_c_;
	std::vector<ParticleRigidBodyCollisionConstraint> particle_rb_collision_c_;
	std::vector<BoxBoundaryConstraint> box_boundary_c_;

	std::vector<PBDParticle> particles_;
	std::vector<int> fluid_particle_idxs_;
	std::vector<PBDRigidBodyParticleData> rb_particles_data_;
	std::vector<PBDFluidParicleRuntimeData> fluid_particles_data_;  // do we need this?
	std::vector<PBDRigidBody> rigid_bodies_;
	std::vector<PBDFluidModel> fluid_models_;
	std::vector<float> inv_scaled_mass_;
	std::vector<vec2> dp_;
	std::vector<int> num_constraints_;
	std::vector<vec2> x_pred_;
    NeighbourData neigh_data_;


    // relaxation parameter, see eq. 10 or 11 in the fluid paper
	inline static const float e_ = 0.0001f;//1.0e-6f;

    float W_zero_;
	float (*kernel_fptr_)(const vec3 &);
	vec3 (*grad_kernel_fptr_)(const vec3& r);

    float W(const vec2& v) const {
        return kernel_fptr_(vec3(v.x, v.y, 0.0f));
    }

    vec2 gradW(const vec2& v) const {
        vec3 grad = grad_kernel_fptr_(vec3(v.x, v.y, 0.0f));
        return vec2(grad.x, grad.y);
    }

    float Wzero() const { return W_zero_; }

};

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim) {
    PBDUnifiedSimulation* sim = new PBDUnifiedSimulation();
    sim->world_size_ = sim_dim;

    float fluid_support_r = sim->particle_r_ * 3.0f;
	CubicKernel::setRadius(fluid_support_r);
	CubicKernel2D::setRadius(fluid_support_r);
	Poly6Kernel::setRadius(fluid_support_r);
	Poly6Kernel2D::setRadius(fluid_support_r);
    SpikyKernel::setRadius(fluid_support_r);
    SpikyKernel2D::setRadius(fluid_support_r);

    sim->kernel_fptr_ = Poly6Kernel2D::W;
	sim->W_zero_ = Poly6Kernel2D::W_zero();
	//sim->grad_kernel_fptr_ = SpikyKernel2D::gradW;
	sim->grad_kernel_fptr_ = Poly6Kernel2D::gradW;

	sim->box_boundary_c_.push_back(BoxBoundaryConstraint{
		//.b_invert = false,
		.p_min = vec2(0),
		.p_max = sim_dim,
        .mu_s = 0.7f,
        .mu_k = 0.4f,
	});
	return sim;
}

void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim) {
    delete sim;
}

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								  float density) {

    float inv_mass = 1.0f / (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
        .flags = PBDParticleFlags::kSolid,
        .phase = -1,
        .rb_data_idx = -1,
        .mu_s = 0.0f,
        .mu_k = 0.0f,
	});

	sim->inv_scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});

    return (int)(sim->particles_.size() - 1);
}

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								 vec2 init_vel, float density) {

	int idx = pbd_unified_sim_add_particle(sim, pos, density);
    sim->particles_[idx].v = init_vel;
    return idx;
}


void pbd_unified_sim_rb_add_velocity(struct PBDUnifiedSimulation* sim, int rb_idx, const vec2& vel) {
    assert((int)sim->rigid_bodies_.size() > rb_idx);
    const int start = sim->rigid_bodies_[rb_idx].start_part_idx;
    const int end = start + sim->rigid_bodies_[rb_idx].num_part;
	for (int i = start; i != end; ++i) {
        int idx = sim->rb_particles_data_[i].index;
        sim->particles_[idx].v += vel;
	}
}

void pbd_unified_sim_particle_set_friction(struct PBDUnifiedSimulation* sim, int idx, float mu_s, float mu_k) {
    assert((int)sim->particles_.size() > idx);
    sim->particles_[idx].mu_s = mu_s;
    sim->particles_[idx].mu_k = mu_k;
}

int pbd_unified_sim_add_rb_particle_data(struct PBDUnifiedSimulation* sim, vec2 x0,
										 float sdf_value, vec2 sdf_grad,
										 int particle_index, bool b_is_boundary,
										 float density) {

	sim->rb_particles_data_.emplace_back(PBDRigidBodyParticleData{
		.x0 = x0,
        .sdf_value = sdf_value,
        .sdf_grad = sdf_grad,
        .index = particle_index,
        .b_is_boundary = b_is_boundary,
	});

    return (int)(sim->rb_particles_data_.size() - 1);
}

void pbd_unified_sim_reset(PBDUnifiedSimulation* sim) {
	sim->particles_.resize(0);
	sim->inv_scaled_mass_.resize(0);
	sim->dp_.resize(0);
	sim->num_constraints_.resize(0);
	sim->x_pred_.resize(0);

    sim->neigh_data_.neighbour_info_.resize(0);
}

float pbd_unified_sim_get_particle_radius(const PBDUnifiedSimulation* sim) {
    return sim->particle_r_;
}

int pbd_unified_sim_get_particle_count(const PBDUnifiedSimulation* sim) {
    return (int)sim->particles_.size();
}
const PBDParticle* pbd_unified_sim_get_particles(const PBDUnifiedSimulation* sim) {
    return sim->particles_.data();
}

static vec3 sdgBox2D(vec2 p, vec2 b )
{
    vec2 w = abs(p)-b;
    vec2 s = vec2(p.x<0.0?-1:1,p.y<0.0?-1:1);
    float g = max(w.x,w.y);
    vec2  q = max(w,vec2(0.0));
    float l = length(q);
	vec2 grad = s * ((g > 0.0) ? q / l : ((w.x > w.y) ? vec2(1, 0) : vec2(0, 1)));
	return vec3((g > 0.0) ? l : g, grad.x, grad.y);
}

int pbd_unified_sim_add_box_rigid_body(struct PBDUnifiedSimulation* sim, int size_x,
										int size_y, vec2 pos, float rot_angle, float density) {

	assert(size_x > 0 && size_y > 0);

	sim->rigid_bodies_.push_back(PBDRigidBody{
		.x = pos,
		.angle = rot_angle,
		.angle0 = rot_angle,
        .mu_s = 0.5f,
        .mu_k = 0.3f,
		.start_part_idx = -1,
		.num_part = size_x * size_y,
		.flags = 0,
	});
    const int rb_idx = (int)(sim->rigid_bodies_.size() - 1);

	const float r = sim->particle_r_;
    const vec2 rb_size = 2.0f * r * vec2(size_x, size_y);
	const vec2 center = 0.5f * rb_size;

    int start_idx = -1;
    for(int y=0;y<size_y;++y) {
        for(int x=0;x<size_x;++x) {

			vec2 x0 = (vec2(r) + vec2(x, y) * 2 * r) - center;
			// in local coords (center is 0,0 no rotation, etc)
			vec3 dist_grad = sdgBox2D(x0, 0.5f * rb_size*vec2(1,0.98f));

            x0 = rotate2(rot_angle) * x0;
			int idx = pbd_unified_sim_add_particle(sim, pos + x0, density);
            vec2 grad_norm = normalize(dist_grad.yz());
			bool b_is_boundary = y == 0 || x == 0 || y == size_y - 1 || x == size_x - 1;
			int data_idx = pbd_unified_sim_add_rb_particle_data(
				sim, x0, dist_grad.x, grad_norm, idx, b_is_boundary, density);

            sim->particles_[idx].flags |= PBDParticleFlags::kRigidBody;
            sim->particles_[idx].phase = rb_idx;
            sim->particles_[idx].rb_data_idx = data_idx;

			if(x==0 && y==0) {
                start_idx = data_idx;
            }

        }
    }

    sim->rigid_bodies_[rb_idx].start_part_idx = start_idx;

    sim->shape_matching_c_.push_back(ShapeMatchingConstraint{rb_idx});

    return rb_idx;
}

int pbd_unified_sim_add_fluid_model(struct PBDUnifiedSimulation* sim, float viscosity, float desired_rest_density) {

    float mass = 1;
    float density0 = 0;

    // calculate rest density for the mass of 1 
    vec2 center(0);
    for(int y=-3; y< 3;++y) {
        for(int x=-3; x< 3;++x) {
            vec2 pos = center + vec2(x,y)*(2*sim->particle_r_);
            density0 += mass * sim->W(center - pos);
        }
    }

    // figure out mass based on desired deisity
    float particle_mass = desired_rest_density / density0;

	sim->fluid_models_.push_back(PBDFluidModel{
        .density0_ = desired_rest_density,
        .mass_ = particle_mass,
		.viscosity_ = viscosity,
	});

    return (int)sim->fluid_models_.size() - 1;
}

int pbd_unified_sim_add_fluid_particle(struct PBDUnifiedSimulation* sim, vec2 pos, int fluid_model_idx) {

    assert((int)sim->fluid_models_.size() > fluid_model_idx);
    const PBDFluidModel& fm = sim->fluid_models_[fluid_model_idx];

    sim->fluid_particles_data_.push_back(PBDFluidParicleRuntimeData());
    const int fluid_particle_data_idx = (int)sim->fluid_particles_data_.size() - 1;

    float inv_mass = 1.0f/fm.mass_;
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
        .flags = PBDParticleFlags::kFluid,
        .phase = fluid_model_idx,
        .fluid_data_idx = fluid_particle_data_idx,
	});

	sim->inv_scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});

    const int added_idx = (int)(sim->particles_.size() - 1);
    sim->fluid_particle_idxs_.push_back(added_idx);

    return added_idx;
}


// assume all masses are similar
vec2 rb_calc_com(const PBDRigidBody& rb, const vec2* pred_x, const PBDRigidBodyParticleData* rb_data) {
    
    vec2 com = vec2(0,0);
    for(int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        com = com + pred_x[pi];
    }
	com *= 1.0f / (float)rb.num_part;
    return com;
}

mat2 calculateQ(const mat2 m, float* angle) {
	float theta = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1]);

    float s, c;
    sincosf(theta, &s, &c);
    *angle = theta;
    return mat2(c, -s, s, c);
}


void solve_shape_matching_c(const ShapeMatchingConstraint& c,
						  const PBDRigidBodyParticleData* rb_data,
						  PBDUnifiedSimulation* sim) {

	const int rb_index = c.rb_index;
    PBDRigidBody& rb = sim->rigid_bodies_[rb_index];
    const vec2 com = rb_calc_com(rb, sim->x_pred_.data(), sim->rb_particles_data_.data());

    mat2 A(0,0,0,0);
    vec2* x_pred = sim->x_pred_.data();
	for (int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        A = A + outer(x_pred[pi] - com, rb_data[di].x0);
	}

    float delta_angle;
    mat2 Q = calculateQ(A, &delta_angle);
    rb.angle = rb.angle0 + delta_angle;
    // test
    mat2 rot_m = rotate2(rb.angle);
    (void)rot_m;

	for (int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
		vec2 dx = (Q * rb_data[di].x0 + com) - x_pred[pi];
        sim->dp_[pi] += dx;
        sim->num_constraints_[pi]++;
	    //x_pred[pi] += dx;
	}
}

void solve_solid_particle_collision_c(const SolidParticlesCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & PBDParticleFlags::kSolid);
	assert(p0.flags & PBDParticleFlags::kSolid);

    const float x_ij_len = length(c.x_ij);
    const float d = x_ij_len - 2*sim->particle_r_;
    assert(d < 0);

	vec2 gradC = c.x_ij / x_ij_len;
	float inv_mass_i = sim->inv_scaled_mass_[c.idx0];
    float inv_mass_j = sim->inv_scaled_mass_[c.idx1];

    float C = d;
    float s = C / (inv_mass_i + inv_mass_j);
    vec2 dp0 = -s * inv_mass_i * gradC;
    vec2 dp1 = +s * inv_mass_j * gradC;
    sim->dp_[c.idx0] += dp0;
    sim->dp_[c.idx1] += dp1;
     
    const float mu_s = 0.5f*(p0.mu_s + p1.mu_s);
    const float mu_k = 0.5f*(p0.mu_k + p1.mu_k);

    // friction
	vec2 dx = (sim->x_pred_[c.idx0] + dp0 - p0.x) -
			  (sim->x_pred_[c.idx1] + dp1 - p1.x);

	// perp projection
    vec2 dxp = dx - fabsf(dot(dx, gradC))*gradC;
    const float ldxp = length(dxp);
	if (ldxp > mu_s * -d) {
		dxp = dxp * min(mu_k * -d / ldxp, 1.0f);
	}

	sim->dp_[c.idx0] += -inv_mass_i/(inv_mass_i + inv_mass_j) * dxp;
    sim->dp_[c.idx1] += +inv_mass_j/(inv_mass_i + inv_mass_j) * dxp;

	sim->num_constraints_[c.idx0]++;
    sim->num_constraints_[c.idx1]++;
}

void solve_particle_rb_collision_c(const ParticleRigidBodyCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & PBDParticleFlags::kRigidBody);
	assert(!(p1.flags & PBDParticleFlags::kRigidBody));

	int32_t rb_idx = sim->particles_[c.idx0].phase;

	assert(rb_idx >= 0 && rb_idx < (int32_t)sim->rigid_bodies_.size());

	const PBDRigidBody& rb = sim->rigid_bodies_[rb_idx];

	int32_t rb_data_idx = p0.rb_data_idx;

	assert(rb_data_idx >= 0 && rb_data_idx < (int32_t)sim->rb_particles_data_.size());

	PBDRigidBodyParticleData& rb_part_data = sim->rb_particles_data_[rb_data_idx];
	assert(rb_part_data.sdf_value <=
		   0); // otherwise not much sense to have a particle whoose center will be
			   // outside of the rigid body

    vec2 norm;
    // maybe makes sense to always use grad in case rb vs. single particle?
    if(fabsf(rb_part_data.sdf_value) < sim->particle_r_) {
        norm = rotate2(rb.angle) * rb_part_data.sdf_grad;
    } else {
		norm = -normalize(c.x_ij);
	}

    const float d = length(c.x_ij) - (sim->particle_r_ + (-rb_part_data.sdf_value));
    if(d >= 0) {
        return;
    }

	// Unified Particle Physics: one sided normals (Ch. 5 Rigid Bodies)
	if(rb_part_data.b_is_boundary) {
        if(dot(-c.x_ij, norm) < 0) {
            norm = normalize(reflect(-c.x_ij, norm));
        } else {
            norm = normalize(-c.x_ij);
        }
    }

#if MAKE_COLLISIONS_GREAT_AGAIN
    norm = normalize(-c.x_ij);
#endif

    // fluid will always be p1
    // looks like not using scaled mass works a bit better with fluid particles, so maybe use it 
	const bool b_with_fluid = p1.flags & PBDParticleFlags::kFluid;

    vec2 gradC = -norm;
    float w0 = (false&&b_with_fluid) ? p0.inv_mass : sim->inv_scaled_mass_[c.idx0];
    float w1 = (false&&b_with_fluid) ? p1.inv_mass : sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    vec2 dp0 = -s * w0 * gradC;
    vec2 dp1 = s * w1 * gradC;
    sim->dp_[c.idx0] += dp0;
    sim->dp_[c.idx1] += dp1;

    // no friction for collision with fluid particle (maybe enable later)
	if (!b_with_fluid) {

		const float mu_s = 0.5f * (p0.mu_s + p1.mu_s);
		const float mu_k = 0.5f * (p0.mu_k + p1.mu_k);

		// friction
		vec2 dx =
			(sim->x_pred_[c.idx0] + dp0 - p0.x) - (sim->x_pred_[c.idx1] + dp1 - p1.x);

		// perp projection
		vec2 dxp = dx - fabsf(dot(dx, gradC)) * gradC;
		const float ldxp = length(dxp);
		if (ldxp > mu_s * -d) {
			dxp = dxp * min(mu_k * -d / ldxp, 1.0f);
		}

		sim->dp_[c.idx0] += -w0 / (w0 + w1) * dxp;
		sim->dp_[c.idx1] += +w1 / (w0 + w1) * dxp;
	}

	sim->num_constraints_[c.idx0]++;
    sim->num_constraints_[c.idx1]++;
}

void solve_rb_collision_c(const RigidBodyCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(p0.flags & PBDParticleFlags::kRigidBody);

	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());
	assert(p1.flags & PBDParticleFlags::kRigidBody);

	int32_t rb0_idx = p0.phase;
	int32_t rb1_idx = p1.phase;

	assert(rb0_idx >= 0 && rb0_idx < (int32_t)sim->rigid_bodies_.size());
	assert(rb1_idx >= 0 && rb1_idx < (int32_t)sim->rigid_bodies_.size());
    assert(rb0_idx != rb1_idx);

	const PBDRigidBody& rb0 = sim->rigid_bodies_[rb0_idx];
	const PBDRigidBody& rb1 = sim->rigid_bodies_[rb1_idx];

	int32_t rb_data_idx0 = p0.rb_data_idx;
	int32_t rb_data_idx1 = p1.rb_data_idx;

	assert(rb_data_idx0 >= 0 && rb_data_idx0 < (int32_t)sim->rb_particles_data_.size());
	assert(rb_data_idx1 >= 0 && rb_data_idx1 < (int32_t)sim->rb_particles_data_.size());

	PBDRigidBodyParticleData& rb_part_data0 = sim->rb_particles_data_[rb_data_idx0];
    PBDRigidBodyParticleData& rb_part_data1 = sim->rb_particles_data_[rb_data_idx1];

    const float d0 = -rb_part_data0.sdf_value;
    const float d1 = -rb_part_data1.sdf_value;

	float d = length(c.x_ij) - (d0 + d1);
	// no rb collision
    if(d >= 0) {
        return;
    }

    vec2 norm;
    if(fabsf(rb_part_data0.sdf_value) < fabsf(rb_part_data1.sdf_value)) {
        norm = rotate2(rb0.angle) * rb_part_data0.sdf_grad;
    } else {
		norm = -rotate2(rb1.angle) * rb_part_data1.sdf_grad;
	}

	// Unified Particle Physics: one sided normals (Ch. 5 Rigid Bodies)
	if(rb_part_data0.b_is_boundary) {
        if(dot(-c.x_ij, norm) < 0) {
            norm = normalize(reflect(-c.x_ij, norm));
        } else {
            norm = normalize(-c.x_ij);
        }
    }

#if MAKE_COLLISIONS_GREAT_AGAIN
    norm = normalize(-c.x_ij);
#endif

    vec2 gradC = -norm;
    float w0 = sim->inv_scaled_mass_[c.idx0];
    float w1 = sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    vec2 dp0 = -s * w0 * gradC;
    vec2 dp1 = s * w1 * gradC;
    sim->dp_[c.idx0] += dp0;
    sim->dp_[c.idx1] += dp1;

    const float mu_s = 0.5f*(p0.mu_s + p1.mu_s);
    const float mu_k = 0.5f*(p0.mu_k + p1.mu_k);

    // friction
	vec2 dx = (sim->x_pred_[c.idx0] + dp0 - p0.x) -
			  (sim->x_pred_[c.idx1] + dp1 - p1.x);

	// perp projection
    vec2 dxp = dx - fabsf(dot(dx, gradC))*gradC;
    const float ldxp = length(dxp);
	if (ldxp > mu_s * -d) {
		dxp = dxp * min(mu_k * -d / ldxp, 1.0f);
	}

	sim->dp_[c.idx0] += -w0/(w0 + w1) * dxp;
    sim->dp_[c.idx1] += +w1/(w0 + w1) * dxp;

	sim->num_constraints_[c.idx0]++;
    sim->num_constraints_[c.idx1]++;
}

void solve_box_boundary(const BoxBoundaryConstraint& c, vec2 pos, int particle_idx, PBDUnifiedSimulation* sim) {

	const float r = sim->particle_r_;
    vec2 dx = vec2(0);
    vec2 n;

    const int prev_constraints = sim->num_constraints_[particle_idx];

    // case when width or height is less than a particle radius is not handled
    // we can change 'else if' for just 'if' but then still it would not work

	if (pos.x - r < c.p_min.x) {

        dx.x = c.p_min.x - (pos.x - r);
        sim->num_constraints_[particle_idx]++;
        n = vec2(-1,0);

    } else if (pos.x + r > c.p_max.x) {

        dx.x = c.p_max.x - (pos.x + r);
        sim->num_constraints_[particle_idx]++;
        n = vec2(1,0);
    }

    if (pos.y - r < c.p_min.y) {

        dx.y = c.p_min.y - (pos.y - r);
        sim->num_constraints_[particle_idx]++;
        n = vec2(0,-1);

    } else if (pos.y + r > c.p_max.y) {

        dx.y = c.p_max.y - (pos.y + r);
        sim->num_constraints_[particle_idx]++;
        n = vec2(0,1);
    }

    // n - points to the gradient increase

    sim->dp_[particle_idx] += dx;

    const bool is_fluid = sim->particles_[particle_idx].flags & PBDParticleFlags::kFluid;
	if (sim->num_constraints_[particle_idx] > prev_constraints && !is_fluid) {

		const float mu_s = 0.5f * (c.mu_s + sim->particles_[particle_idx].mu_s);
        const float mu_k = 0.5f * (c.mu_k + sim->particles_[particle_idx].mu_k);

		float d = length(dx);

		// friction
		vec2 dxp = (sim->x_pred_[particle_idx] + dx) - sim->particles_[particle_idx].x;
		// perp projection, I think abs is necessary here
		dxp = dxp - fabsf(dot(dxp, n)) * n;
		const float ldxp = length(dxp);
		if (ldxp > mu_s * d) {
			dxp = dxp * min(mu_k * d / ldxp, 1.0f);
		}
		sim->dp_[particle_idx] -= dxp;
	}
}

////////////////////////////////////////////////////////////////////////////////
class PBDUnifiedTimestep {
	static const int sim_iter_count = 5;
	static const int stab_iter_count = 0;
	static const constexpr vec2 g = vec2(0.0f, -9.8f);
	static const constexpr float k = 1.5f;
	static const constexpr float eps = 1e-6f;
    // if movement less than 1% of particle radius
	static const constexpr float kSleepEpsSq = (0.01f * 0.1f) * (0.01f * 0.1f);
    // successive over-relaxation coefficient
	static const constexpr float kSOR = 1.0f; // [1,2]

	static const constexpr float kCFL = 0.4f;

	static float h(vec2 pos) { return pos.y; }

  public:
	void Simulate(PBDUnifiedSimulation* sim, float dt);
    void fluidUpdate(PBDUnifiedSimulation* sim, float dt, std::vector<vec2>& pos_array);
    void fluidUpdate_GS(PBDUnifiedSimulation* sim, float dt, const std::vector<vec2>& pos_array);
    float getBoundaryDensity(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array);
    vec2 getBoundaryGrad(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array);

};

void pbd_unified_timestep(PBDUnifiedSimulation* sim, float dt) {
    PBDUnifiedTimestep uts;
    uts.Simulate(sim, dt);
}

void PBDUnifiedTimestep::Simulate(PBDUnifiedSimulation* sim, float dt) {

    dt = 0.016f;

	const int num_particles = (int)sim->particles_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	std::vector<float>& scaled_mass = sim->inv_scaled_mass_;
	std::vector<vec2>& x_pred = sim->x_pred_;

	// predict
	for (int i = 0; i < num_particles; i++) {
		p[i].v = p[i].v + dt * g;
		x_pred[i] = p[i].x + dt * p[i].v;
		scaled_mass[i] = p[i].inv_mass * exp(k * h(p[i].x)); // h(x_pred[i]) ?
	}

	findNeighboursNaiive(sim->x_pred_.data(), (int)sim->particles_.size(),
						 sim->support_r_, sim->particles_.data(), &sim->neigh_data_);


    const float r = sim->particle_r_;
	std::vector<vec2>& dp = sim->dp_;

	// handle only contact constraints in stabilization phase
	for (int iter = 0; iter < stab_iter_count; iter++) {

        sim->rb_collision_c_.resize(0);
        sim->particle_rb_collision_c_.resize(0);
        sim->solid_collision_c_.resize(0);
	    for (int i = 0; i < num_particles; i++) {
            sim->num_constraints_[i] = 0;
            sim->dp_[i] = vec2(0);
        }

		// TODO: this is wrong: should pass p[i].x, but it should be factored out in separate array for
		// this to make easy
		fluidUpdate(sim, dt, x_pred);

		for (int i = 0; i < num_particles; i++) {

            for(const BoxBoundaryConstraint& c: sim->box_boundary_c_) {
                solve_box_boundary(c, p[i].x, i, sim); 
            }

			const auto& neigh_list = sim->neigh_data_.neigh_idx_;
			const int n_start = sim->neigh_data_.neighbour_info_[i].start;
			const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				if (j < i) {
					continue;
				}

				vec2 vec = p[i].x - p[j].x;
				float dist = length(vec);

				float C = dist - 2*r;
				if (C < 0) {

                    if((p[i].flags&p[j].flags) & PBDParticleFlags::kRigidBody) {
                        sim->rb_collision_c_.push_back(RigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[i].flags & PBDParticleFlags::kRigidBody) {
                        sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[j].flags & PBDParticleFlags::kRigidBody) {
                        sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{j,i, -vec});
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kSolid) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                        //solve_solid_particle_collision_c(SolidParticlesCollisionConstraint{i,j, vec}, sim);
                    } else if((p[i].flags & PBDParticleFlags::kSolid) && (p[j].flags & PBDParticleFlags::kFluid)) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                    } else if((p[j].flags & PBDParticleFlags::kSolid) && (p[i].flags & PBDParticleFlags::kFluid)) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kFluid) {
                        // handled separately in updateFluid
                    } else {
                        assert(0 && "unsupported contact configuration");
                    }
				}
			}

		}

        for(auto c: sim->solid_collision_c_) {
            solve_solid_particle_collision_c(c, sim);
        }

        for(auto c: sim->rb_collision_c_) {
            solve_rb_collision_c(c, sim);
        }

        for(auto c: sim->particle_rb_collision_c_) {
            solve_particle_rb_collision_c(c, sim);
        }

        // not necessary in pre-sim step?
        for(auto c: sim->shape_matching_c_) {
            solve_shape_matching_c(c, sim->rb_particles_data_.data(), sim);
        }

        for (int i = 0; i < num_particles; i++) {
            //  adjust x* = x* + dx;
            // could set num_constraints to 1 during initialization to avoid branch
            if (sim->num_constraints_[i]) {
                vec2 dp_i = (kSOR / sim->num_constraints_[i]) * dp[i];
                p[i].x += dp_i;
                x_pred[i] += dp_i;
            }
        }
	}

    // solve constraints 
	for (int iter = 0; iter < sim_iter_count; iter++) {

        sim->rb_collision_c_.resize(0);
        sim->particle_rb_collision_c_.resize(0);
        sim->solid_collision_c_.resize(0);
	    for (int i = 0; i < num_particles; i++) {
            sim->num_constraints_[i] = 0;
            sim->dp_[i] = vec2(0);
        }

        fluidUpdate(sim, dt, x_pred);

		for (int i = 0; i < num_particles; i++) {

            for(const BoxBoundaryConstraint& c: sim->box_boundary_c_) {
                if(0 == (p[i].flags & PBDParticleFlags::kFluid)) {
                    solve_box_boundary(c, x_pred[i], i, sim); 
                }
            }

			const auto& neigh_list = sim->neigh_data_.neigh_idx_;
			const int n_start = sim->neigh_data_.neighbour_info_[i].start;
			const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				// because we project constraint for 2 particles
				if (j < i) {
					continue;
				}

				vec2 vec = x_pred[i] - x_pred[j];
				float dist_sqr = length(vec);

				float C = dist_sqr - 2*r;
				if (C < 0) {
                    if(((p[i].flags&p[j].flags) & PBDParticleFlags::kRigidBody) ) {
                        if(p[i].rb_data_idx!=p[j].rb_data_idx)
                            sim->rb_collision_c_.push_back(RigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[i].flags & PBDParticleFlags::kRigidBody) {
                            sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[j].flags & PBDParticleFlags::kRigidBody) {
                            sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{j,i, -vec});
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kSolid ) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                        //solve_solid_particle_collision_c(SolidParticlesCollisionConstraint{i,j, vec}, sim);
                    } else if((p[i].flags & PBDParticleFlags::kSolid) && (p[j].flags & PBDParticleFlags::kFluid)) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                    } else if((p[j].flags & PBDParticleFlags::kSolid) && (p[i].flags & PBDParticleFlags::kFluid)) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{j,i, -vec});
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kFluid) {
                        // handled separately in updateFluid
                    } else {
                        assert(0 && "unsupported contact configuration");
                    }
				}
			}

		}

        for(auto c: sim->solid_collision_c_) {
            solve_solid_particle_collision_c(c, sim);
        }

        for(auto c: sim->rb_collision_c_) {
            solve_rb_collision_c(c, sim);
        }

        for(auto c: sim->particle_rb_collision_c_) {
            solve_particle_rb_collision_c(c, sim);
        }

        for(auto c: sim->shape_matching_c_) {
            solve_shape_matching_c(c, sim->rb_particles_data_.data(), sim);
        }

        for (int i = 0; i < num_particles; i++) {
            //  adjust x* = x* + dx;
            // could set num_constraints to 1 during initialization to avoid branch
            if (sim->num_constraints_[i]) {
                x_pred[i] = x_pred[i] + (kSOR / sim->num_constraints_[i]) * dp[i];
            }
        }
	}

	for (int i = 0; i < num_particles; i++) {
        vec2 dpv = x_pred[i] - p[i].x;
        float dp_len = length(dpv);
#if ENABLE_CFL
        // CFL
        const float max_vel_ = kCFL * sim->particle_r_;
        if(dp_len > max_vel_) {
            dpv *= max_vel_ / dp_len;
            p[i].x = p[i].x + dpv;
        }
#endif

		// update velocity (first order)
		p[i].v = dpv / dt;

		// update position or sleep
        bool b_sleep = dp_len < kSleepEpsSq;
        if(!b_sleep) {
		    p[i].x = x_pred[i];
            p[i].flags &= ~PBDParticleFlags::kSleep;
        } else {
            p[i].flags |= PBDParticleFlags::kSleep;
        }
	}
}

float PBDUnifiedTimestep::getBoundaryDensity(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array) {

	std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
    const float h = 3*sim->particle_r_;
    const float r = sim->particle_r_;
    const float m = 1.0f/p[i].inv_mass;

    vec2 pos = pos_array[i];
    float density = 0;
	for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {

		if (pos.x - h < c.p_min.x) {
            density += m * sim->W(x[i] - vec2(c.p_min.x - r, x[i].y));
		} else if (pos.x + h > c.p_max.x) {
            density += m * sim->W(x[i] - vec2(c.p_max.x + r, x[i].y));
		}

		if (pos.y - h < c.p_min.y) {
            density += m * sim->W(x[i] - vec2(x[i].x, c.p_min.y - r));
		} else if (pos.y + h > c.p_max.y) {
            density += m * sim->W(x[i] - vec2(x[i].x, c.p_max.y + r));
		}
	}

    return density;
}

vec2 PBDUnifiedTimestep::getBoundaryGrad(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array) {

	const std::vector<vec2>& x = pos_array;
    const float r_eps = 0.001f;
    const float h = 3*sim->particle_r_ + r_eps;
    const float r = sim->particle_r_;

    vec2 pos = pos_array[i];
    vec2 grad = vec2(0);
    float contacts = 0;
	for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {

		if (pos.x - h <= c.p_min.x) {
            grad += sim->gradW(x[i] - vec2(c.p_min.x - r, x[i].y));
            contacts++;
		} else if (pos.x + h >= c.p_max.x) {
            grad += sim->gradW(x[i] - vec2(c.p_max.x + r, x[i].y));
            contacts++;
		}

		if (pos.y - h <= c.p_min.y) {
            grad += sim->gradW(x[i] - vec2(x[i].x, c.p_min.y - r));
            contacts++;
		} else if (pos.y + h >= c.p_max.y) {
            grad += sim->gradW(x[i] - vec2(x[i].x, c.p_max.y + r));
            contacts++;
		}
	}

    //return contacts!=0 ? grad/contacts : vec2(0.0f);
    return grad;
}

void PBDUnifiedTimestep::fluidUpdate(PBDUnifiedSimulation* sim, float dt,
									 std::vector<vec2>& pos_array) {

    return fluidUpdate_GS(sim, dt, pos_array);

	const int num_fluid_particles = (int)sim->fluid_particle_idxs_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
	const auto& neigh_list = sim->neigh_data_.neigh_idx_;
	const float bs = 0.0f;

	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {
			solve_box_boundary(c, x[i], i, sim);
		}
	}

	for (int fi = 0; fi < num_fluid_particles; fi++) {

		const int i = sim->fluid_particle_idxs_[fi];
		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		const float fmass = 1.0f / p[i].inv_mass;
		float density = sim->Wzero() * fmass;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			float s = 1.0f;
			if (0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				s = 1.0f;
			}

			// all particles participate in density calculations according to Eq. 27
			float m = 1.0f / p[j].inv_mass;
			//m = min(m, fmass);
			density += s * m * sim->W(x[i] - x[j]);
		}

		density += bs * getBoundaryDensity(sim, i, pos_array);

		sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = 0;
		const float mi = 1.0f / p[i].inv_mass;
		const float C = density / fm.density0_ - 1;
		if (C > 0)
        {
			float sum_grad_C_sq = 0;
			vec2 grad_Ci = vec2(0, 0); // first case when k = i

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				//if (0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				//	continue;
				//}

				// looks like Eq. (7) and (8) miss mass multiplier
				float m = 1.0f / p[j].inv_mass;
				vec2 grad_Cj = -(m / fm.density0_) * sim->gradW(x[i] - x[j]);
				sum_grad_C_sq += lengthSqr(grad_Cj);
				grad_Ci += -grad_Cj;
			}

			if (1) {
				vec2 b_grad_Cj =
					-bs * (mi / fm.density0_) * getBoundaryGrad(sim, i, pos_array);
				sum_grad_C_sq += lengthSqr(b_grad_Cj);
				grad_Ci += -b_grad_Cj;
			}

			float sum_grad_sq = (sum_grad_C_sq + lengthSqr(grad_Ci));
			float lambda = -C / (sum_grad_sq + sim->e_);

			sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = lambda;
		}
	}

	// solve fluid constraint
	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		vec2 dp = vec2(0, 0);
		const float lambda_i = sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			if(0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				// probably, should not influence fluid from rigid particles as fluid is
				// not influencing them as well
				//dp += (m / fm.density0_) * (lambda_i)*sim->gradW(x[i] - x[j]);
                continue;
            }

            assert(p[j].flags & PBDParticleFlags::kFluid);
            float lambda_j = sim->fluid_particles_data_[p[j].fluid_data_idx].lambda_;
            dp += (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i + lambda_j) *
                sim->gradW(x[i] - x[j]);
		}

		// boundary
		float b_density = bs * getBoundaryDensity(sim, i, pos_array);
		vec2 b_grad = bs * getBoundaryGrad(sim, i, pos_array);
		if (b_density) {
			dp += bs * (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i)*b_grad;
		}

		sim->num_constraints_[i]++;
		sim->dp_[i] += dp;
	}

	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		assert(sim->num_constraints_[i]);
		pos_array[i] += (kSOR / sim->num_constraints_[i]) * sim->dp_[i];
		sim->num_constraints_[i] = 0;
		sim->dp_[i] = vec2(0);
	}
}

// so calles Gauss-Seidel style, when we update position right after calculating it, this
// make algorithm serial, but I think we can still parallelize this using atomics
void PBDUnifiedTimestep::fluidUpdate_GS(PBDUnifiedSimulation* sim, float dt,
									 const std::vector<vec2>& pos_array) {

	const int num_fluid_particles = (int)sim->fluid_particle_idxs_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
	const auto& neigh_list = sim->neigh_data_.neigh_idx_;
	const float bs = 0.0f;

	for (int fi = 0; fi < num_fluid_particles; fi++) {

		const int i = sim->fluid_particle_idxs_[fi];

        solve_box_boundary(sim->box_boundary_c_[0], sim->x_pred_[i], i, sim);
        sim->x_pred_[i] = sim->x_pred_[i] + sim->dp_[i];
        sim->num_constraints_[i] = 0;
        sim->dp_[i] = vec2(0);

		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		const float fmass = 1.0f / p[i].inv_mass;
		float density = sim->Wzero() * fmass;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			// all particles participate in density calculations according to Eq. 27
			float s = (p[j].flags & PBDParticleFlags::kFluid) ? 1.0f : 1.0f;
            
			// do not select min mass as (instead use real) it prevents simulation to
			// converge (looks like)
			float m = 1.0f / p[j].inv_mass;
			density += s * m * sim->W(x[i] - x[j]);
		}

		density += bs * getBoundaryDensity(sim, i, pos_array);

		sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = 0;
		const float C = density / fm.density0_ - 1;
		const float mi = 1.0f / p[i].inv_mass;
		vec2 grad_Ci = vec2(0, 0); // first case when k = i
		if (C > 0)
        {
			float sum_grad_C_sq = 0;

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				float m = 1.0f / p[j].inv_mass;
				vec2 grad_Cj = -(m / fm.density0_) * sim->gradW(x[i] - x[j]);
				sum_grad_C_sq += lengthSqr(grad_Cj);
				grad_Ci += -grad_Cj;
			}

			if (1) {
				vec2 b_grad_Cj =
					-bs * (mi / fm.density0_) * getBoundaryGrad(sim, i, pos_array);
				sum_grad_C_sq += lengthSqr(b_grad_Cj);
				grad_Ci += -b_grad_Cj;
			}

			float sum_grad_sq = (sum_grad_C_sq + lengthSqr(grad_Ci));
			float lambda = -C / (sum_grad_sq + sim->e_);

			sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = lambda;
		}

		const float lambda_i = sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			if(0 == (p[j].flags & PBDParticleFlags::kFluid)) {
                // special behaviour for solid particles
                // would be better if actually use SDF for RBs normal calculation
                const float dist = length(x[i] - x[j]);
                const float d = max(0.0f, (2*sim->particle_r_ - dist));
                const float wi = 0 ? sim->inv_scaled_mass_[i] : p[i].inv_mass;
                const float wj = 0 ? sim->inv_scaled_mass_[j] : p[j].inv_mass;
			    sim->x_pred_[i] = sim->x_pred_[i] + (wi/(wi+wj)) * d * normalize(x[i] - x[j]);
			    sim->x_pred_[j] = sim->x_pred_[j] - (wj/(wi+wj)) * d * normalize(x[i] - x[j]);
            } 
			// TBH, having "else" is more correct, but adding pressure from fluid to all particles
			// (not only fluid ones) makes sim better
            /*else*/
            {
				// for some precision reason using gr2 form instead of gr makes simulation
				// behave worse with a specially crafted scene
				// scene_fluid_simple_expose_bug, this could be fixed by multiplying gr by
				// some number close to 1.0 (e.g. 0.99999f) though. Anyhow it is sad that
				// difference in 0.0000001 can make a difference.
				vec2 gr =
					-(1.0f / (p[i].inv_mass * fm.density0_)) * sim->gradW(x[i] - x[j]);
#if 0
                vec2 gr2 = -(mi / fm.density0_) * sim->gradW(x[i] - x[j]);
                if (fabs(gr.x - gr2.x) > gr_eps) {
                    //printf("gr.x: %.8f gr2.x: %.8f\n", gr.x, gr2.x);
                }
                if (fabs(gr.y - gr2.y) > gr_eps) {
                    //printf("gr.y: %.8f gr2.y: %.8f\n", gr.y, gr2.y);
                }
#endif
                sim->x_pred_[j] = sim->x_pred_[j] + lambda_i * gr;
            }
		}

		sim->x_pred_[i] = sim->x_pred_[i] + lambda_i * grad_Ci;

		// boundary
		float b_density = bs * getBoundaryDensity(sim, i, pos_array);
		vec2 b_grad = bs * getBoundaryGrad(sim, i, pos_array);
		if (b_density) {
            const vec2 dp = bs * (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i)*b_grad;
		    sim->x_pred_[i] = sim->x_pred_[i] + dp;
		}

        assert(sim->num_constraints_[i] == 0);
        assert(sim->dp_[i] == vec2(0));
	}
}

const PBDRigidBodyParticleData* pbd_unified_sim_get_rb_particle_data(const struct PBDUnifiedSimulation* sim) {
    return sim->rb_particles_data_.data();
}

const PBDRigidBody* pbd_unified_sim_get_rigid_bodies(const struct PBDUnifiedSimulation* sim) {
    return sim->rigid_bodies_.data();
}

vec2 pbd_unified_sim_get_world_bounds(const struct PBDUnifiedSimulation* sim) {
    return sim->world_size_;
}
