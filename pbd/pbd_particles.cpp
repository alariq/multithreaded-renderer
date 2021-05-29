// Unified Particle Physics for Real-Time Applications

#include "pbd_particles.h"

#include <cmath>
#include <vector>
#include <cassert>

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

			// phase should be different or -1, maybe leave "j" in neighbours but just use
			// this condition when checking constraints?
			bool b_phase_cond = (particles[i].phase != particles[j].phase) || (particles[i].phase == -1);
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
	std::vector<PBDRigidBodyParticleData> rb_particles_data_;
	std::vector<PBDRigidBody> rigid_bodies_;
	std::vector<float> inv_scaled_mass_;
	std::vector<vec2> dp_;
	std::vector<int> num_constraints_;
	std::vector<vec2> x_pred_;
    NeighbourData neigh_data_;
};

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim) {
    PBDUnifiedSimulation* sim = new PBDUnifiedSimulation();
    sim->world_size_ = sim_dim;
	sim->box_boundary_c_.push_back(BoxBoundaryConstraint{
		//.b_invert = false,
		.p_min = vec2(0),
		.p_max = sim_dim,
	});
	return sim;
}

void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim) {
    delete sim;
}

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								  float density) {

    float inv_mass = 1.0f / sim->particle_r_ * sim->particle_r_ * density;
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
        .flags = PBDParticleFlags::kSolid,
        .phase = -1,
        .rb_data_idx = -1,
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

    mat2 Q = calculateQ(A, &rb.angle);
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

	assert(sim->particles_[c.idx0].flags & PBDParticleFlags::kSolid);
	assert(sim->particles_[c.idx1].flags & PBDParticleFlags::kSolid);

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
     
    float mu = 0.4f;

    // friction
	vec2 dx = (sim->x_pred_[c.idx0] + dp0 - sim->particles_[c.idx0].x) -
			  (sim->x_pred_[c.idx1] + dp1 - sim->particles_[c.idx1].x);

	// perp projection
    vec2 dxp = dx - dot(dx, gradC)*gradC;
    const float ldxp = length(dxp);
	if (ldxp > mu * -d) {
		dxp = dxp * min(mu * -d / ldxp, 1.0f);
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
	assert(sim->particles_[c.idx0].flags & PBDParticleFlags::kRigidBody);
	assert(!(sim->particles_[c.idx1].flags & PBDParticleFlags::kRigidBody));

	int32_t rb_idx = sim->particles_[c.idx0].phase;

	assert(rb_idx >= 0 && rb_idx < (int32_t)sim->rigid_bodies_.size());

	const PBDRigidBody& rb = sim->rigid_bodies_[rb_idx];

	int32_t rb_data_idx = sim->particles_[c.idx0].rb_data_idx;

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

    vec2 gradC = -norm;
    float w0 = sim->inv_scaled_mass_[c.idx0];
    float w1 = sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    sim->dp_[c.idx0] += -s * w0 * gradC;
    sim->dp_[c.idx1] += s * w1 * gradC;

	sim->num_constraints_[c.idx0]++;
    sim->num_constraints_[c.idx1]++;
}

void solve_rb_collision_c(const RigidBodyCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(sim->particles_[c.idx0].flags & PBDParticleFlags::kRigidBody);

	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());
	assert(sim->particles_[c.idx1].flags & PBDParticleFlags::kRigidBody);

	int32_t rb0_idx = sim->particles_[c.idx0].phase;
	int32_t rb1_idx = sim->particles_[c.idx1].phase;

	assert(rb0_idx >= 0 && rb0_idx < (int32_t)sim->rigid_bodies_.size());
	assert(rb1_idx >= 0 && rb1_idx < (int32_t)sim->rigid_bodies_.size());

	const PBDRigidBody& rb0 = sim->rigid_bodies_[rb0_idx];
	const PBDRigidBody& rb1 = sim->rigid_bodies_[rb1_idx];

	int32_t rb_data_idx0 = sim->particles_[c.idx0].rb_data_idx;
	int32_t rb_data_idx1 = sim->particles_[c.idx1].rb_data_idx;

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

    vec2 gradC = -norm;
    float w0 = sim->inv_scaled_mass_[c.idx0];
    float w1 = sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    sim->dp_[c.idx0] += -s * w0 * gradC;
    sim->dp_[c.idx1] += s * w1 * gradC;

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
        n = vec2(0,1);

    } else if (pos.y + r > c.p_max.y) {

        dx.y = c.p_max.y - (pos.y + r);
        sim->num_constraints_[particle_idx]++;
        n = vec2(0,-1);
    }

    sim->dp_[particle_idx] += dx;

	if (sim->num_constraints_[particle_idx] > prev_constraints) {

		float mu = 0.2f;
		float d = length(dx);

		// friction
		vec2 dxp = (sim->x_pred_[particle_idx] + dx) - sim->particles_[particle_idx].x;
		// perp projection
		dxp = dxp - dot(dxp, n) * n;
		const float ldxp = length(dxp);
		if (ldxp > mu * d) {
			dxp = dxp * min(mu * d / ldxp, 1.0f);
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
	static const constexpr float sleep_eps_sq = (0.01f * 0.1f) * (0.01f * 0.1f);
    // successive over-relaxation coefficient
	static const constexpr float kSOR = 1.0f; // [1,2]

	static float h(vec2 pos) { return pos.y; }

  public:
	void Simulate(PBDUnifiedSimulation* sim, float dt);
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
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kSolid ) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                        //solve_solid_particle_collision_c(SolidParticlesCollisionConstraint{i,j, vec}, sim);
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

		for (int i = 0; i < num_particles; i++) {

            for(const BoxBoundaryConstraint& c: sim->box_boundary_c_) {
                solve_box_boundary(c, x_pred[i], i, sim); 
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
                    if((p[i].flags&p[j].flags) & PBDParticleFlags::kRigidBody) {
                        sim->rb_collision_c_.push_back(RigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[i].flags & PBDParticleFlags::kRigidBody) {
                        sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{i,j, vec});
                    } else if(p[j].flags & PBDParticleFlags::kRigidBody) {
                        sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{j,i, -vec});
                    } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kSolid ) {
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                        //solve_solid_particle_collision_c(SolidParticlesCollisionConstraint{i,j, vec}, sim);
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
		// update velocity
		p[i].v = (x_pred[i] - p[i].x) / dt;
		// update position or sleep
        bool b_sleep = lengthSqr(p[i].x - x_pred[i]) < sleep_eps_sq;
        if(!b_sleep) {
		    p[i].x = x_pred[i];
            p[i].flags &= ~PBDParticleFlags::kSleep;
        } else {
            p[i].flags |= PBDParticleFlags::kSleep;
        }
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
