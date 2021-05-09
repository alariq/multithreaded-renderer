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

    vec2 word_size = vec2(5,5);

	std::vector<ShapeMatchingConstraint> shape_matching_constraints_;

	std::vector<PBDParticle> particles_;
	std::vector<PBDRigidBodyParticleData> rb_particles_data_;
	std::vector<PBDRigidBody> rigid_bodies_;
	std::vector<float> scaled_mass_;
	std::vector<vec2> dp_;
	std::vector<int> num_constraints_;
	std::vector<vec2> x_pred_;
    NeighbourData neigh_data_;
};

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim) {
    PBDUnifiedSimulation* sim = new PBDUnifiedSimulation();
    sim->word_size = sim_dim;
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
        .flags = 0,
        .phase = -1,
	});

	sim->scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});

    return (int)(sim->particles_.size() - 1);
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
	sim->scaled_mass_.resize(0);
	sim->dp_.resize(0);
	sim->num_constraints_.resize(0);
	sim->x_pred_.resize(0);

    sim->neigh_data_.neighbour_info_.resize(0);
}

float pbd_unified_sim_get_particle_radius(struct PBDUnifiedSimulation* sim) {
    return sim->particle_r_;
}

int pbd_unified_sim_get_particle_count(PBDUnifiedSimulation* sim) {
    return (int)sim->particles_.size();
}
const PBDParticle* pbd_unified_sim_get_particles(PBDUnifiedSimulation* sim) {
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
										int size_y, vec2 pos, float rot, float density) {

	assert(size_x > 0 && size_y > 0);

	sim->rigid_bodies_.push_back(PBDRigidBody{
		.x = pos,
		.angle = 0,
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
            x0 = rotate2(rot) * x0;
			int idx = pbd_unified_sim_add_particle(sim, pos + x0, density);
            sim->particles_[idx].flags |= PBDParticleFlags::kRigidBody;
            sim->particles_[idx].phase = rb_idx;

			vec3 dist_grad = sdgBox2D(x0, 0.5f * rb_size);
			bool b_is_boundary = y == 0 || x == 0 || y == size_y - 1 || x == size_x - 1;
			int data_idx = pbd_unified_sim_add_rb_particle_data(
				sim, x0, dist_grad.x, dist_grad.xy(), idx, b_is_boundary, density);

			if(x==0 && y==0) {
                start_idx = data_idx;
            }

        }
    }

    sim->rigid_bodies_[rb_idx].start_part_idx = start_idx;

    sim->shape_matching_constraints_.push_back(ShapeMatchingConstraint{rb_idx});

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

mat2 calculateQ(const mat2 m) {
	float theta = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1]);

    float s, c;
    sincosf(theta, &s, &c);
    return mat2(c, -s, s, c);
}


void solve_shape_matching(const ShapeMatchingConstraint& c,
						  const PBDRigidBodyParticleData* rb_data,
						  PBDUnifiedSimulation* sim) {

	const int rb_index = c.rb_index;
    const PBDRigidBody& rb =sim->rigid_bodies_[rb_index];
    const vec2 com = rb_calc_com(rb, sim->x_pred_.data(), sim->rb_particles_data_.data());

    mat2 A(0,0,0,0);
    vec2* x_pred = sim->x_pred_.data();
	for (int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        A = A + outer(x_pred[pi] - com, rb_data[di].x0);
	}

    mat2 Q = calculateQ(A);

	for (int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
		vec2 dx = (Q * rb_data[di].x0 + com) - x_pred[pi];
	    x_pred[pi] += dx;
        sim->num_constraints_[pi]++;
	}
}

////////////////////////////////////////////////////////////////////////////////
class PBDUnifiedTimestep {
	static const int sim_iter_count = 15;
	static const int stab_iter_count = 0;
	static const constexpr vec2 g = vec2(0.0f, -9.8f);
	static const constexpr float k = 1.5f;
	static const constexpr float eps = 1e-6f;
    // if movement less than 1% of particle radius
	static const constexpr float sleep_eps_sq = (0.01f * 0.1f) * (0.01f * 0.1f);
    // successive over-relaxation coefficient
	static const constexpr float kSOR = 1.5f; // [1,2]

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
	std::vector<float>& scaled_mass = sim->scaled_mass_;
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
    const float r_sqr = r * r;
	// handle only contact constraints in stabilization phase
	for (int iter = 0; iter < stab_iter_count; iter++) {

	    for (int i = 0; i < num_particles; i++) {
            sim->num_constraints_[i] = 0;
            sim->dp_[i] = vec2(0);
        }

		for (int i = 0; i < num_particles; i++) {
			vec2 dx(0);
			vec2 pos = p[i].x;

			if (pos.x - r < 0) {
				dx.x = 0 - (pos.x - r);
				sim->num_constraints_[i]++;
			}
			if (pos.x + r > sim->word_size.x) {
				dx.x = sim->word_size.x - (pos.x + r);
				sim->num_constraints_[i]++;
			}
			if (pos.y - r < 0) {
				dx.y = 0 - (pos.y - r);
				sim->num_constraints_[i]++;
			}
			if (pos.x + r > sim->word_size.x) {
				dx.x = sim->word_size.x - (pos.x + r);
				sim->num_constraints_[i]++;
			}

			const auto& neigh_list = sim->neigh_data_.neigh_idx_;
			const int n_start = sim->neigh_data_.neighbour_info_[i].start;
			const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				vec2 vec = p[i].x - p[j].x;
				float dist_sqr = lengthSqr(vec);

				float C = dist_sqr - 4*r_sqr;
				if (C < 0) {
					vec2 gradC = vec / sqrt(dist_sqr);
                    float inv_mass_i = scaled_mass[i];//p[i].inv_mass;
                    float inv_mass_j = scaled_mass[j];//p[j].inv_mass;

					float s = C / (inv_mass_i + inv_mass_j);
					vec2 dx_i = -s * inv_mass_i * gradC;
					dx += dx_i;
					sim->num_constraints_[i]++;
				}
			}

            for(auto c: sim->shape_matching_constraints_) {
                solve_shape_matching(c, sim->rb_particles_data_.data(), sim);
            }

            if(sim->num_constraints_[i]) {
                vec2 dp = dx / sim->num_constraints_[i];
				p[i].x += dp;
				x_pred[i] += dp;
            }
		}
	}

    // solve constraints 
	std::vector<vec2>& dp = sim->dp_;
	for (int iter = 0; iter < sim_iter_count; iter++) {

	    for (int i = 0; i < num_particles; i++) {
            sim->num_constraints_[i] = 0;
            sim->dp_[i] = vec2(0);
        }

		for (int i = 0; i < num_particles; i++) {

            const vec2 pos = x_pred[i];

			if (pos.x - r < 0) {
				dp[i].x += 0 - (pos.x - r);
				sim->num_constraints_[i]++;
			}
			if (pos.x + r > sim->word_size.x) {
				dp[i].x = sim->word_size.x - (pos.x + r);
				sim->num_constraints_[i]++;
			}
			if (pos.y - r < 0) {
				dp[i].y += 0 - (pos.y - r);
				sim->num_constraints_[i]++;
			}
			if (pos.x + r > sim->word_size.x) {
				dp[i].x += sim->word_size.x - (pos.x + r);
				sim->num_constraints_[i]++;
			}
            
            //dp[i] *= 2.0f; // overrelaxation

			if(0 && sim->num_constraints_[i]) {
                x_pred[i] += 2.0f * dp[i];
                dp[i] = vec2(0);
                sim->num_constraints_[i]--;
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
				float dist_sqr = lengthSqr(vec);

				float C = dist_sqr - 4*r_sqr;
				if (C < 0) {
					vec2 gradC = vec / sqrt(dist_sqr);
                    float inv_mass_i = scaled_mass[i];//p[i].inv_mass;
                    float inv_mass_j = scaled_mass[j];//p[j].inv_mass;

					float s = C / (inv_mass_i + inv_mass_j);
					vec2 dx_i = -s * inv_mass_i * gradC;
					vec2 dx_j = +s * inv_mass_j * gradC;

					dp[i] += dx_i;
					dp[j] += dx_j;
					sim->num_constraints_[i]++;
					sim->num_constraints_[j]++;

                    //x_pred[i] += dp[i];
                    //dp[i] = vec2(0);
					//sim->num_constraints_[i]--;

                    //x_pred[j] += dp[j];
                    //dp[j] = vec2(0);
					//sim->num_constraints_[j]--;
				}
			}

            for(auto c: sim->shape_matching_constraints_) {
                solve_shape_matching(c, sim->rb_particles_data_.data(), sim);
            }
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
