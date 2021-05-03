// Unified Particle Physics for Real-Time Applications

#include "pbd_particles.h"

#include <cmath>
#include <vector>

struct Pair {
	int start;
	int num;
};

struct NeighbourData {
	std::vector<int> neigh_idx_;
	std::vector<Pair> neighbour_info_;
};

void findNeighboursNaiive(const vec2* x, const int size, float radius,
						  NeighbourData* nd) {
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

			if (lengthSqr(x_i - x[j]) < r_sq) {
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

	std::vector<PBDParticle> particles_;
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

void pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								  float density) {

    float inv_mass = 1.0f / sim->particle_r_ * sim->particle_r_ * density;
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
	});

	sim->scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});
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

////////////////////////////////////////////////////////////////////////////////
class PBDUnifiedTimestep {
	static const int sim_iter_count = 5;
	static const int stab_iter_count = 5;
	static const constexpr vec2 g = vec2(0.0f, -9.8f);
	static const constexpr float k = 1.0f;
	static const constexpr float eps = 1e-6f;
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
		scaled_mass[i] = p[i].inv_mass * exp(k * h(x_pred[i]));
	}

	findNeighboursNaiive(sim->x_pred_.data(), (int)sim->particles_.size(),
						 sim->support_r_, &sim->neigh_data_);


    const float r = sim->particle_r_;
    const float r_sqr = r * r;
	// handle only contact constraints in stabilization phase
	for (int iter = 0; iter < stab_iter_count; iter++) {
		for (int i = 0; i < num_particles; i++) {
			vec2 dx(0);
			vec2 pos = p[i].x;

			if (pos.x - r < 0) {
				dx.x = 0 - (pos.x - r);
			}
			if (pos.x + r > sim->word_size.x) {
				dx.x = sim->word_size.x - (pos.x + r);
			}
			if (pos.y - r < 0) {
				dx.y = 0 - (pos.y - r);
			}
			if (pos.x + r > sim->word_size.x) {
				dx.x = sim->word_size.x - (pos.x + r);
			}

            p[i].x += dx;
            x_pred[i] += dx;
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
					float s = C / (p[i].inv_mass + p[j].inv_mass);
					vec2 dx_i = -s * p[i].inv_mass * gradC;
					vec2 dx_j = +s * p[j].inv_mass * gradC;

					dp[i] += dx_i;
					dp[j] += dx_j;
					sim->num_constraints_[i]++;
					sim->num_constraints_[j]++;
				}
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
		// update position (TODO: sleep)
		p[i].x = x_pred[i];
	}
}
