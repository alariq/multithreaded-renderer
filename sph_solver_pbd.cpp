#include "sph_solver_pbd.h"
#include "sph.h"
#include "sph_boundary.h"
#include "engine/gameos.hpp"

#include "utils/vec.h"
#include <cfloat>

struct SPHSimData_PBD: public SPHSimData {
    std::vector<float> lambda_;
    std::vector<vec3> dx_;
    std::vector<vec3> old_x_;
    std::vector<vec3> prev_x_;

    static constexpr const char* const type_ = "SPHSimData_PBD";

    // from boundary (overkill because only a fraction of particles needs this during a simulation frame)
    // use index into another array (resized on demand, or using fixed block allocator scheme)
    // even better, store this in boundaries sim data (does not extist yet)
	struct bi_t {
		std::vector<vec2> boundaryXj_;
		std::vector<float> boundaryVolume_;
		std::vector<int> boundaryIdx_;
	};

	std::vector<bi_t> b_;

    virtual const char* getType() const override { return type_; }

	virtual void allocate(size_t count) override {
		lambda_.resize(count);
		dx_.resize(count);
		old_x_.resize(count);
		prev_x_.resize(count);

		b_.resize(count);
	}

	virtual void init(const struct SPHParticle2D* p, int idx) override {
		old_x_[idx] = vec3(p->pos, 0.0f);
        prev_x_[idx] = old_x_[idx];
	}

	virtual int append(size_t count) override {
        auto old_count = lambda_.size();
        auto new_count = old_count + count;
        allocate(new_count);
        return new_count - 1;
    }

    virtual ~SPHSimData_PBD() = default;
};

// Position Based Fluids http://mmacklin.com/pbf_slides.pdf
// http://mmacklin.com/pbf_sig_preprint.pdf
struct PBD_TimeStep {

	inline static const float s_eps_ = 1.0e-5f;
    // relaxation parameter, see eq. 9 or 10 in the paper
	inline static const float s_e_ = 0.0001f;//1.0e-6f;

    const int min_iterations_ = 10;//2;
    const int max_iterations_ = 100;
    const float max_error_percent_ = 0.01f;

    int getClosestBoundary(const vec2& pos, const SPHSimulation* sim, SPHBoundaryModel **bm, float* dist, vec2* normal) {
        *bm = nullptr;
        *dist = FLT_MAX;
        *normal = vec2(0,0);
        int index = -1;
        for(int i=0; i< (int)sim->boundary_models_.size(); ++i) {
            SPHBoundaryModel* model = sim->boundary_models_[i];

            float cur_dist = model->getDistance2D(pos);
            if(cur_dist < *dist) {
                *dist = cur_dist;
                *normal = model->getNormal2D(pos);
                *bm = model;
                index = i;
            }
        }
        return index;
    }

    void checkCollisions(vec2& pos, const SPHSimulation* sim) {
        for(SPHBoundaryModel* model : sim->boundary_models_) {
            float dist = model->getDistance2D(pos);
            if(dist < 0) {
                vec2 normal = model->getNormal2D(pos);
                pos -= normal*dist;
            }
        }
    }

	void calcVolumeAndBoundaryX(SPHSimulation* sim, SPHSimData_PBD* sim_data) {
		SPHFluidModel* fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D* particles = fm->particles_.data();
		const float particle_radius = fm->radius_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];

			sim_data->b_[i].boundaryIdx_.clear();
			sim_data->b_[i].boundaryVolume_.clear();
			sim_data->b_[i].boundaryXj_.clear();

			checkCollisions(pi.pos, sim);

			bool b_enable_boundary_vol = true;
			if (b_enable_boundary_vol) {
				SPHBoundaryModel* boundary;
				float dist;
				vec2 normal;
				// int index = getClosestBoundary(pi.pos, sim, &boundary, &dist, &normal);
				// if (!boundary) continue;

				for (int bi = 0; bi < (int)sim->boundary_models_.size(); ++bi) {
					boundary = sim->boundary_models_[bi];
					dist = boundary->getDistance2D(pi.pos);
					normal = boundary->getNormal2D(pi.pos);

					dist = max(0.0f, dist);

					if ((dist > 0.1f * particle_radius) && (dist < support_radius)) {
						const float volume = boundary->getVolume2D(pi.pos);
						if ((volume > 1e-5) && (volume != FLT_MAX)) {
							sim_data->b_[i].boundaryIdx_.push_back(bi);
							sim_data->b_[i].boundaryVolume_.push_back(volume);
							sim_data->b_[i].boundaryXj_.push_back(pi.pos - normal * dist);
						}
					} else if (dist <= 0.1 * particle_radius) {
						// float d = -dist;
						// d = min(d, (0.25f / 0.005f) * particle_radius *
						// sim->time_step_);
						float d = dist < 0 ? -dist : (0.5 * particle_radius - dist);
						// bring back to boundary surface
						pi.pos += d * normal;
						// adapt velocity in normal direction
						pi.vel += (0.05f - dot(pi.vel, normal)) * normal;
						// compensate for the gravity
						//+ sim->accel_*sim->time_step_;
					}
				}
			}
		}
	}

	void calcDensities(SPHSimulation *sim, SPHSimData_PBD* sim_data) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			pi.density = fm->volume_ * sim->Wzero();
			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&pi, fm, sim](const SPHParticle2D *pj, int) {
								  pi.density += fm->volume_ * sim->W(pi.pos - pj->pos);
							  });

            // boundary
            for(int bi = 0; bi< (int)sim_data->b_[i].boundaryIdx_.size(); bi++) {
                float Vj = sim_data->b_[i].boundaryVolume_[bi];
                if( Vj > 0.0f) {
                    vec2 xj = sim_data->b_[i].boundaryXj_[bi];
                    vec2 d = pi.pos - xj;
                    float k = sim->W(d);
                    pi.density += Vj * k;
                }
            }

			// multiply by density because we use volume in the loop instead of a mass
			pi.density *= fm->density0_;
		}
	}

	void calcNonPressureForces(SPHSimulation* sim, SPHSimData_PBD* sim_data) {
		SPHFluidModel* fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D* particles = fm->particles_.data();
		SPHGrid* grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		// apply viscosity
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];

			vec2 avg_vel = vec2(0.0f);
			float num = 0.0f;
			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&avg_vel, &num](const SPHParticle2D* pj, int j) {
								  avg_vel += pj->vel;
								  num++;
							  });
            if(0 == num) {
                continue;
            }

            // viscosity kernel (Laplacian) is propotional to (l - r)
            // SPH Based Shallow Water Simulation (eq. 9, 10)
			vec2 dv = avg_vel - pi.vel;
			pi.vel += dv * fm->viscosity_;
		}
	}

	// returns average density error
	float projectConstraintsIter(SPHSimulation *sim, SPHSimData_PBD* sim_data, const float dt, int iter) {

		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;
		//const float h = sim->time_step_;

        calcVolumeAndBoundaryX(sim, sim_data);

        float density_err = 0.0;

        // calc lambda
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
            sim_data->lambda_[i] = 0.0f;
            // relative to density0
            float density = fm->volume_ * sim->Wzero();

			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&pi, fm, sim, &density](const SPHParticle2D* pj, int j) {
								  density += fm->volume_ * sim->W(pi.pos - pj->pos);
							  });

			for (int bi = 0; bi < (int)sim_data->b_[i].boundaryIdx_.size(); bi++) {
				float Vj = sim_data->b_[i].boundaryVolume_[bi];
				if (Vj > 0.0f) {
					vec2 xj = sim_data->b_[i].boundaryXj_[bi];
					vec2 d = pi.pos - xj;
					float k = sim->W(d);
					density += Vj * k;
				}
			}

			density_err += max(density, 1.0f) - 1.0f;


            // evaluate costraint function
            // max to prevent clumping at borders
            // TODO: add Artificial Pressure as described in the paper
            const float C = max(density - 1.0f, 0.0f);

            sim_data->lambda_[i] = 0.0f;
			if (C != 0.0f) { // does it even make sense? eps? or remove check at all
                // Eq.8 from the paper

				float sum_grad_C_sq = 0.0f;
				vec2 grad_Ci = vec2(0, 0);
				foreach_in_radius(support_radius, i, grid, particles, num_particles,
								  [&pi, fm, sim, &grad_Ci,
								   &sum_grad_C_sq](const SPHParticle2D* pj, int) {
									  vec2 grad_Cj =
										  -fm->volume_ * sim->gradW(pi.pos - pj->pos);
									  sum_grad_C_sq += lengthSqr(grad_Cj);
									  grad_Ci -= grad_Cj;
								  });
				// boundary
                for (int bi = 0; bi < (int)sim_data->b_[i].boundaryIdx_.size(); bi++) {
					float Vj = sim_data->b_[i].boundaryVolume_[bi];
					if (Vj > 0.0f) {
						vec2 xj = sim_data->b_[i].boundaryXj_[bi];
						vec2 grad_Cj = -Vj * sim->gradW(pi.pos - xj);
						grad_Ci -= grad_Cj;
					}
				}

				sum_grad_C_sq += lengthSqr(grad_Ci);

                sim_data->lambda_[i] = -C / (sum_grad_C_sq + s_e_);
			}
        }

        // solve density constraint

		// we store delta pos in temporary dx variable and apply later, this is because we
		// are using Jacobi style solver which updates all constraints in parallel and
		// then applies results ( even though we are not parallelizing anything here, we
		// can/will in future)
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];
			sim_data->dx_[i] = vec3(0.0f);

			foreach_in_radius(
				support_radius, i, grid, particles, num_particles,
				[i, pi, fm, sim, sim_data](const SPHParticle2D* pj, int j) {
					vec2 grad_Cj = -fm->volume_ * sim->gradW(pi.pos - pj->pos);
					// TODO: to support fluid models with different densities
					// float k = density[j]/density0;
					float k = 1.0f;
					vec2 dx = (k * sim_data->lambda_[j] + sim_data->lambda_[i]) * grad_Cj;
					// TODO: seamless 2D/3D
					sim_data->dx_[i] -= vec3(dx, 0);
				});

			// boundary
			const float lambda_i = sim_data->lambda_[i];
			for (int bi = 0; bi < (int)sim_data->b_[i].boundaryIdx_.size(); bi++) {
				float Vj = sim_data->b_[i].boundaryVolume_[bi];
				if (Vj > 0.0f && lambda_i) {
					vec2 xj = sim_data->b_[i].boundaryXj_[bi];
					vec2 grad_Cj = -Vj * sim->gradW(pi.pos - xj);
					vec2 dx = lambda_i * grad_Cj;
					sim_data->dx_[i] -= vec3(dx, 0);

					auto boundary =
						sim->boundary_models_[sim_data->b_[i].boundaryIdx_[bi]];
					if (boundary->isDynamic()) {
						float mass = fm->density0_ * fm->volume_;
						boundary->addForce(vec3(xj, 0),
										   (mass) * vec3(dx, 0) / (dt * dt));
					}
				}
			}
		}

		// apply dx
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];
			// TODO: check for static / dynamic particles
			// TODO: seamless 2D/3D
			pi.pos += sim_data->dx_[i].xy();
		}

		return density_err / num_particles;
	}

	void projectConstraints(SPHSimulation* sim, SPHSimData_PBD* sim_data) {

        int num_iterations = 0;

        //calcVolumeAndBoundaryX(sim, sim_data);
#if 0
		SPHFluidModel* fm = sim->fluid_model_;
		float density0 = fm->density0_;
		bool check = false;
		const float eta = (max_error_percent_ * 0.01f) * density0;

		while ((!check || (num_iterations < min_iterations_)) &&
			   num_iterations < max_iterations_) {
			check = true;
			float avg_density_err = projectConstraintsIter(sim, sim_data);
			check = check && (avg_density_err < eta);
			num_iterations++;
		}
#endif
        const float h = sim->time_step_;
		while (num_iterations < min_iterations_) {
			projectConstraintsIter(sim, sim_data, h, num_iterations);
			num_iterations++;
		}

		printf("num iterations: %d\n", num_iterations);

    }

	void Tick(SPHSimulation *sim) {

		SPHFluidModel *fm = sim->fluid_model_;
        assert(fm->sim_data_->isOfType(SPHSimData_PBD::type_));
        SPHSimData_PBD* sim_data = (SPHSimData_PBD*)fm->sim_data_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();

        // use constant timestep (no CFL update)
        //const float h = sim->time_step_;
        //const float oo_h = 1.0f/h;


        const int num_iter = 1;
        for(int iter=0;iter<num_iter;++iter) {
            const float h = sim->time_step_ / num_iter;
            const float oo_h = 1.0f/h;


        // predict new positions
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];

            sim_data->old_x_[i] = sim_data->prev_x_[i];
            sim_data->prev_x_[i] = vec3(pi.pos, 0);

            // semi implicit Euler
            pi.vel += h * sim->accel_;
            pi.pos += h * pi.vel;
		}

        // perform neighbours search
        // TODO:

        //solve density constraint
        projectConstraints(sim, sim_data);

	    const float max_vel_ = 0.4f * fm->radius_;
        // update velocities
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D& pi = particles[i];
            vec2 dp = pi.pos - sim_data->prev_x_[i].xy();
            float dp_len = length(dp);
            // CFL
            if(dp_len > max_vel_) {
                dp *= max_vel_ / dp_len;
                pi.pos = sim_data->prev_x_[i].xy() + dp;
            }

            // first order
            pi.vel = oo_h * dp;
            // second order
            //pi.vel = oo_h * (1.5f * pi.pos - 2.0f * sim_data->prev_x_[i].xy() + 0.5f * sim_data->old_x_[i].xy());
        }

        // do we need this ?
        //calcDensities(sim, sim_data);

        // Apply XSPH Viscosity + Vorticity Confinement
        calcNonPressureForces(sim, sim_data);

        }

	}

};

void SPH_PBDDebugDrawSimData(const SPHFluidModel* fm, RenderList* rl) {

    assert(fm->sim_data_->isOfType(SPHSimData_PBD::type_));
	SPHSimData_PBD* sim_data = (SPHSimData_PBD*)fm->sim_data_;

	for (int i = 0; i < (int)fm->particles_.size(); ++i) {

		for (int bi = 0; bi < (int)sim_data->b_[i].boundaryIdx_.size(); bi++) {
			float Vj = sim_data->b_[i].boundaryVolume_[bi];
			if (Vj != 0.0f) {
				vec3 p = sim_data->prev_x_[i];
				p.z = -0.05f;
                rl->addDebugLine(p, vec3(sim_data->b_[i].boundaryXj_[bi], -0.05f), vec4(1, 1, 1, 1));
				//gos_AddLine(p, vec3(sim_data->b_[i].boundaryXj_[bi], -0.05f), vec4(1, 1, 1, 1));
			}
		}
	}
}

SPHSimData* SPH_PBDCreateSimData() {
    return new SPHSimData_PBD();
}

void SPH_PBDTimestepTick(SPHSimulation* sim, float dt)
{
    static PBD_TimeStep df_sph_ts;
    df_sph_ts.Tick(sim);
}

const struct SPHSolverInterface* Get_SPH_PBD_SolverInterface() {
	const static SPHSolverInterface si = {SPH_PBDCreateSimData, SPH_PBDTimestepTick};
	return &si;
}

