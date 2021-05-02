#include "sph_solver_df.h"
#include "sph.h"
#include "sph_boundary.h"
#include "engine/gameos.hpp"

#include "utils/vec.h"
#include <cfloat>

struct SPHSimData_DF: public SPHSimData {
    std::vector<float> kappa_;
    std::vector<float> factor_;
    std::vector<float> density_adv_;

    static constexpr const char* const type_ = "SPHSimData_DF";

    // from boundary (overkill because only a fraction of particles needs this dunring a simulation frame)
    // use index into another array (resized on demand, or using fixed block allocator scheme)
    std::vector<vec2> boundaryXj_;
    std::vector<float> boundaryVolume_;
    std::vector<vec2> prev_pos_;

    virtual const char* getType() const override { return type_; }

	virtual void allocate(size_t count) override {
		density_adv_.resize(count);
		factor_.resize(count);
		kappa_.resize(count);
		boundaryXj_.resize(count);
		boundaryVolume_.resize(count);
		prev_pos_.resize(count);
	}

	virtual int append(size_t count) override {
        auto old_count = density_adv_.size();
        auto new_count = old_count + count;
        allocate(new_count);
        return new_count - 1;
    }

    virtual ~SPHSimData_DF() = default;
};

// Divergence-Free SPH for Incompressible and Viscous Fluids
struct DF_TimeStep {

	inline static const float s_eps_ = 1.0e-15f;

    const int min_iterations_ = 2;
    const int max_iterations_ = 100;
    const float max_error_percent_ = 0.01f;

    void getClosestBoundary(const vec2& pos, const SPHSimulation* sim, SPHBoundaryModel **bm, float* dist, vec2* normal) {
        *bm = nullptr;
        *dist = FLT_MAX;
        *normal = vec2(0,0);
        for(SPHBoundaryModel* model : sim->boundary_models_) {
            float cur_dist = model->getDistance2D(pos);
            if(cur_dist < *dist) {
                *dist = cur_dist;
                *normal = model->getNormal2D(pos);
                *bm = model;
            }
        }
    }

	void calcVolumeAndBoundaryX(SPHSimulation *sim, SPHSimData_DF* sim_data) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		const float particle_radius = fm->radius_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];

			sim_data->boundaryVolume_[i] = 0.0f;
            sim_data->prev_pos_[i] = pi.pos;
           
            SPHBoundaryModel* boundary;
			float dist;
            vec2 normal;
            getClosestBoundary(pi.pos, sim, &boundary, &dist, &normal);
            if(!boundary)
                continue;

            //dist = std::max(0,dist);

			if ((dist > 0.1 * particle_radius) && (dist < support_radius)) {
				const float volume = boundary->getVolume2D(pi.pos);
				if ((volume > 1e-5) && (volume != FLT_MAX)) {
					sim_data->boundaryVolume_[i] = volume;
					sim_data->boundaryXj_[i] = pi.pos - normal*(dist + 0*0.5f*particle_radius);
				}
			} else if(dist <= 0.1*particle_radius) {
                float d = -dist;
				d = min(d, (0.25f / 0.005f) * particle_radius * sim->time_step_);
				//float d = dist < 0 ? -dist : (0.5*particle_radius - dist);
                // bring back to boundary surface
                pi.pos += d * normal;
                // adapt velocity in normal direction
				pi.vel += (0.05f - dot(pi.vel, normal)) * normal ;
                    // compensate for the gravity
                    //+ sim->accel_*sim->time_step_;
			}
		}
	}

	void calcDensities(SPHSimulation *sim, SPHSimData_DF* sim_data) {
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
            float Vj = sim_data->boundaryVolume_[i];
            if( Vj > 0.0f) {
                vec2 xj = sim_data->boundaryXj_[i];
                vec2 d = pi.pos - xj;
                float k = sim->W(d);
                pi.density += Vj * k;
            }

			// multiply by density because we use volume in the loop instead of a mass
			pi.density *= fm->density0_;
		}
	}

	void calcFactor(SPHSimulation *sim, SPHSimData_DF* sim_data) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			float sum_of_grad_squares = 0.0f;
			vec2 grad_sums = vec2(0, 0);
			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&pi, fm, sim, &grad_sums,
							   &sum_of_grad_squares](const SPHParticle2D *pj, int) {
								  vec2 grad_pj =
									  -fm->volume_ * sim->gradW(pi.pos - pj->pos);
								  grad_sums -= grad_pj;
								  sum_of_grad_squares += lengthSqr(grad_pj);
							  });
            // boundary
            float Vj = sim_data->boundaryVolume_[i];
            if( Vj > 0.0f) {
				vec2 xj = sim_data->boundaryXj_[i];
				vec2 grad_pj = -Vj * sim->gradW(pi.pos - xj);
				grad_sums -= grad_pj;
				//sum_of_grad_squares += lengthSqr(grad_pj);
            }

			float alpha = lengthSqr(grad_sums) + sum_of_grad_squares;
			if (alpha > s_eps_)
				alpha = -1.0f / alpha;
			else
				alpha = 0.0f;
			sim_data->factor_[i] = alpha;
		}
	}
#if 0
    void clearAccel(SPHSimulation* sim) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
        }
    }
#endif

    void calcNonPressureForces() {}

	// clac "Rho star i" (3.3) more on this in "SPH Fluids in Computer Graphics" Markus
	// Imsen (Ch 3.2) and his "Implicit Incompressible SPH" (Ch 2.1)
	void calcDensityAdv(SPHSimulation *sim, SPHSimData_DF* sim_data) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			float delta = 0.0f;
			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&pi, fm, sim, &delta](const SPHParticle2D *pj, int) {
								  delta +=
									  fm->volume_ * dot((pi.vel - pj->vel),
														sim->gradW(pi.pos - pj->pos));
							  });
            // boundary
            float Vj = sim_data->boundaryVolume_[i];
            if( Vj > 0.0f) {
                const vec2 vj = vec2(0.0f, 0.0f);// only static boundaries for now
                vec2 xj = sim_data->boundaryXj_[i];
				delta += Vj * dot((pi.vel - vj), sim->gradW(pi.pos - xj));
			}

            float density_adv = pi.density / fm->density0_ + sim->time_step_ * delta;
            sim_data->density_adv_[i] = max(density_adv, 1.0f);
		}

	}

    // returns average density error
	float pressureSolveIter(SPHSimulation *sim, SPHSimData_DF* sim_data) {

		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;
		const float *factor = sim_data->factor_.data();
		const float *density_adv = sim_data->density_adv_.data();
		const float h = sim->time_step_;
        const float oo_h2 = 1.0f/(h*h);

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			// Algorithm 3, see also Ch 3.3 eq. 12
			float kappa_i = (density_adv[i] - 1.0f) * factor[i];

			foreach_in_radius(support_radius, i, grid, particles, num_particles,
							  [&pi, fm, sim, h, oo_h2, kappa_i, factor,
							   density_adv](const SPHParticle2D *pj, int j) {
								  // float rel_dens0 = neighb_fluid_model_density0 /
								  // density0;
								  float rel_dens0 = 1.0f;
								  float kappa_j =
									  (density_adv[j] - rel_dens0) * factor[j];
								  float k_sum = oo_h2*(kappa_i + kappa_j);
								  if (fabs(k_sum) > s_eps_) {
									  vec2 grad_p_j =
										  -fm->volume_ * sim->gradW(pi.pos - pj->pos);
									  pi.vel -= h * k_sum * grad_p_j;
								  }
							  });
            // boundary
			if (fabs(kappa_i*oo_h2) > s_eps_) {
				float Vj = sim_data->boundaryVolume_[i];
				if (Vj > 0.0f) {
                        const vec2 xj = sim_data->boundaryXj_[i];
						const vec2 grad_p_j = -Vj * sim->gradW(pi.pos - xj);
						const vec2 vel_change = -sim->time_step_ * 1.0f * kappa_i * oo_h2 * grad_p_j;				// kj already contains inverse density
						pi.vel += vel_change;
					// static boundaries do not affected by force
                    // in case of dynamic boundaries add force
				}
			}
		}

        // now when we've udated velocities, update density advection
	    calcDensityAdv(sim, sim_data);
        // move this into calcDensityAdv
        float density_err = 0.0;
		for (int i = 0; i < num_particles; ++i) {
            density_err += (sim_data->density_adv_[i] - 1.0f) * fm->density0_;
        }
		return density_err / num_particles;
	}

	void pressureSolve(SPHSimulation* sim, SPHSimData_DF* sim_data) {

        calcDensityAdv(sim, sim_data);
        int num_iterations = 0;


		SPHFluidModel *fm = sim->fluid_model_;
        float density0 = fm->density0_;
        bool check = false;
        const float eta = (max_error_percent_ * 0.01f) * density0; 

        while((!check || (num_iterations < min_iterations_)) && num_iterations < max_iterations_) {
            check = true;
            float avg_density_err = pressureSolveIter(sim, sim_data);
            check = check && (avg_density_err < eta);
            num_iterations++;
        }

        printf("num iterations: %d\n", num_iterations);

    }

	void Tick(SPHSimulation *sim) {

		SPHFluidModel *fm = sim->fluid_model_;
        assert(fm->sim_data_->isOfType(SPHSimData_DF::type_));
        SPHSimData_DF* sim_data = (SPHSimData_DF*)fm->sim_data_;
		const int num_particles = (int)fm->particles_.size();
		SPHParticle2D *particles = fm->particles_.data();

        calcVolumeAndBoundaryX(sim, sim_data);

		calcDensities(sim, sim_data);
		calcFactor(sim, sim_data);

        calcNonPressureForces();

        sim->updateTimestep();
        const float h = sim->time_step_;
        printf("timestep h: %.4f\n", h);      

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
            pi.vel += h * sim->accel_;
        }

        pressureSolve(sim, sim_data);

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
            pi.pos += h * pi.vel;
		}
	}

};

SPHSimData* SPH_DFCreateSimData() {
    return new SPHSimData_DF();
}

void SPH_DFTimestepTick(SPHSimulation* sim)
{
    static DF_TimeStep df_sph_ts;
    df_sph_ts.Tick(sim);
}

// this is called from render thread but uses main thread sim_data, so line blinking
// possible if boundaryVolume_[i] changes meanwhile
void SPH_DFDebugDrawSimData(const SPHFluidModel* fm, RenderList* rl) {

    assert(fm->sim_data_->isOfType(SPHSimData_DF::type_));
    SPHSimData_DF* sim_data = (SPHSimData_DF*)fm->sim_data_;

    for(int i=0; i<(int)fm->particles_.size();++i) {
        if(sim_data->boundaryVolume_[i]!=0.0f) {
            rl->addDebugLine(vec3(sim_data->prev_pos_[i], -0.05f), vec3(sim_data->boundaryXj_[i], -0.05f), vec4(1,1,1,1));
        }
    }

}
