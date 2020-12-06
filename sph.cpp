#include "sph.h"
#include "sph_kernels.h"
#include <cmath>
#include "obj_model.h"
#include "res_man.h"
#include "engine/gameos.hpp" // COUNTOF
#include "utils/vec.h"
#include <functional>

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


struct SPHFluidModel {
    SPHGrid* grid_;
    SPHParticle2D* particles_;
    int num_particles_;
    float radius_;
    float density0_;
    float volume_;
    float support_radius_;
};

struct SPHSimData {
    std::vector<float> kappa_;
    std::vector<float> factor_;
    std::vector<float> density_adv_;

};

struct Simulation {

    const vec2 accel_ = vec2(0.0f, -9.81f);

    float time_step_ = 0.016f;
    float cfl_factor_ = 0.5f;
    float min_cfl_timestep_ = 0.0001;
    float max_cfl_timestep_ = 0.016f;//0.005;

    float W_zero_;
	float (*kernel_fptr_)(const vec3 &);
	vec3 (*grad_kernel_fptr_)(const vec3& r);

    void Setup() {
        W_zero_ = CubicKernel::W_zero();
        kernel_fptr_ = CubicKernel::W;
    }

    float W(const vec3& v) const {
        return kernel_fptr_(v);
    }

    float W(const vec2& v) const {
        return kernel_fptr_(vec3(v.x, v.y, 0.0f));
    }

    vec2 gradW(const vec2& v) const {
        vec3 grad = grad_kernel_fptr_(vec3(v.x, v.y, 0.0f));
        return vec2(grad.x, grad.y);
    }

    vec3 gradW(const vec3& v) const {
        return grad_kernel_fptr_(v);
    }

    void updateTimestep() {
        SPHFluidModel* fm = fluid_model_;
        float max_vel = 0.01f;
		for (int i = 0; i < fm->num_particles_; ++i) {
			SPHParticle2D &pi = fm->particles_[i];
            float vel = lengthSqr(pi.vel + accel_*time_step_);
            max_vel = max(max_vel, vel);
        }
        
        float diameter = fm->radius_ * 2.0f;
        float h = cfl_factor_ * diameter / sqrtf(max_vel);
        h = clamp(h, min_cfl_timestep_, max_cfl_timestep_);
        time_step_ = h;
    }

    SPHFluidModel* fluid_model_;
    SPHSimData* sim_data_;
};

// Divergence-Free SPH for Incompressible andViscous Fluids
struct TimeStep {

	const float eps_ = 1.0e-5;

    int num_iterations_ = 0;
    const int min_iterations_ = 2;
    const int max_iterations_ = 100;
    const float max_error_percent_ = 0.01f;

	void calcDensities(Simulation *sim) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			pi.density = 0.0f;
			foreach_in_radius(support_radius, pi.pos, grid, particles, num_particles,
							  [&pi, fm, sim](const SPHParticle2D *pj, int) {
								  pi.density += fm->volume_ * sim->W(pj->pos - pi.pos);
							  });
			// multiply by density because we use volume in the loop instead of a mass
			pi.density *= fm->density0_;
		}
	}

	void calcFactor(Simulation *sim) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;
		SPHSimData *data = sim->sim_data_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			float sum_of_grad_squares = 0.0f;
			vec2 grad_sums = vec2(0, 0);
			foreach_in_radius(support_radius, pi.pos, grid, particles, num_particles,
							  [&pi, fm, sim, &grad_sums,
							   &sum_of_grad_squares](const SPHParticle2D *pj, int) {
								  vec2 grad_pj =
									  -fm->volume_ * sim->gradW(pi.pos - pj->pos);
								  grad_sums -= grad_pj;
								  sum_of_grad_squares += lengthSqr(grad_pj);
							  });

			float alpha = lengthSqr(grad_sums) + sum_of_grad_squares;
			if (alpha > eps_)
				alpha = -1.0f / alpha;
			else
				alpha = 0.0f;
			data->factor_[i] = alpha;
		}
	}
#if 0
    void clearAccel(Simulation* sim) {
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
	void calcDensityAdv(Simulation *sim) {
		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			float delta = 0.0f;
			foreach_in_radius(support_radius, pi.pos, grid, particles, num_particles,
							  [&pi, fm, sim, &delta](const SPHParticle2D *pj, int) {
								  delta +=
									  fm->volume_ * dot((pi.vel - pj->vel),
														sim->gradW(pi.pos - pj->pos));
							  });
            float density_adv = pi.density / fm->density0_ + sim->time_step_ * delta;
            sim->sim_data_->density_adv_[i] = max(density_adv, 1.0f);
		}

	}

    // returns average density error
	float pressureSolveIter(Simulation *sim) {

		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;
		SPHGrid *grid = fm->grid_;
		const float support_radius = fm->support_radius_;
		const float *factor = sim->sim_data_->factor_.data();
		const float *density_adv = sim->sim_data_->density_adv_.data();
		float eps = this->eps_;
		const float h = sim->time_step_;
        const float oo_h2 = 1.0f/(h*h);

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
			// Algorithm 3, see also Ch 3.3 eq. 12
			float kappa_i = (density_adv[i] - 1.0f) * factor[i];

			foreach_in_radius(support_radius, pi.pos, grid, particles, num_particles,
							  [&pi, fm, sim, eps, h, oo_h2, kappa_i, factor,
							   density_adv](const SPHParticle2D *pj, int j) {
								  // float rel_dens0 = neighb_fluid_model_density0 /
								  // density0;
								  float rel_dens0 = 1.0f;
								  float kappa_j =
									  (density_adv[j] - rel_dens0) * factor[j];
								  float k_sum = oo_h2*(kappa_i + kappa_j);
								  if (fabs(k_sum) > eps) {
									  vec2 grad_p_j =
										  -fm->volume_ * sim->gradW(pi.pos - pj->pos);
									  pi.vel -= h * k_sum * grad_p_j;
								  }
							  });
		}

        // now when we've udated velocities, update density advection
	    calcDensityAdv(sim);
        // move this into calcDensityAdv
        float density_err = 0.0;
		for (int i = 0; i < num_particles; ++i) {
            density_err = (sim->sim_data_->density_adv_[i] - 1.0f) * fm->density0_;
        }
		return density_err / num_particles;
	}

	void pressureSolve(Simulation* sim) {

        calcDensityAdv(sim);
        num_iterations_ = 0;


		SPHFluidModel *fm = sim->fluid_model_;
        float density0 = fm->density0_;
        bool check = false;
        const float eta = (max_error_percent_ * 0.01f) * density0; 

        while((!check || (num_iterations_ < min_iterations_)) && num_iterations_ < max_iterations_) {
            check = true;
            float avg_density_err = pressureSolveIter(sim);
            check = check && (avg_density_err < eta);
            num_iterations_++;
        }       

    }

	void Tick(Simulation *sim) {

		SPHFluidModel *fm = sim->fluid_model_;
		const int num_particles = fm->num_particles_;
		SPHParticle2D *particles = fm->particles_;

		calcDensities(sim);
		calcFactor(sim);

        calcNonPressureForces();

        sim->updateTimestep();
        const float h = sim->time_step_;

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
            pi.vel += h * sim->accel_;
        }

        pressureSolve(sim);

		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = particles[i];
            pi.pos += h * pi.vel;
        }
	}

};

void foreach_in_radius(float r, vec2 pos, SPHGrid* grid, SPHParticle2D* particles, int num_particles, std::function<void(const SPHParticle2D*, int)> func) {

    (void)num_particles;

    const int left_top = pos2cell_index(pos - vec2(r,r), grid);
    const int right_bottom = pos2cell_index(pos + vec2(r,r), grid);
    const float r_sq = r * r;
    const ivec2 lt_coord = grid->getCellCoordFromCellIndex(left_top);
    const ivec2 rb_coord = grid->getCellCoordFromCellIndex(right_bottom);

    int n_start_x = max(lt_coord.x - 1, 0);
    int n_end_x = min(rb_coord.x + 1, grid->cell_dim_.x - 1);

    int n_start_y = max(lt_coord.y - 1, 0);
	int n_end_y = min(rb_coord.y + 1, grid->cell_dim_.y - 1);

	for (int ny = n_start_y; ny <= n_end_y; ++ny) {
		for (int nx = n_start_x; nx <= n_end_x; ++nx) {
            const SPHGridCell* cell = grid->at(nx, ny);
            for (int pj_idx : cell->part_indices_) {
                assert(pj_idx < num_particles);
                if(lengthSqr(pos - particles[pj_idx].pos) <= r_sq)
                    func(&particles[pj_idx], pj_idx);
            }
		}
	}
}

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

void ComputeDensityPressure_neigh(SPHParticle2D *particles, int count, SPHGrid *grid) {
	for (int i = 0; i < count; ++i) {
		SPHParticle2D &pi = particles[i];
        pi.density = 0.0f;
		foreach_in_radius(H, pi.pos, grid, particles, count, [&pi](const SPHParticle2D* pj, int) {
			vec2 rij = pj->pos - pi.pos;
			float r2 = lengthSqr(rij);

			if (r2 < HSQ) {
				// this computation is symmetric
				pi.density += MASS * POLY6 * pow(HSQ - r2, 3.f);
			}
		});
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

void ComputeForces_neigh(SPHParticle2D *particles, int count, SPHGrid *grid) {
	for (int i = 0; i < count; ++i) {
		SPHParticle2D* pi = &particles[i];

		vec2 fpress(0.f, 0.f);
		vec2 fvisc(0.f, 0.f);

		foreach_in_radius(H, pi->pos, grid, particles, count, [pi, &fpress, &fvisc](const SPHParticle2D* pj, int) {

			if (pi == pj)
                return;

			vec2 rij = pj->pos - pi->pos;
			float r = length(rij);

			if (r < H) {
				// compute pressure force contribution
				fpress += -normalize(rij) * MASS * (pi->pressure + pj->pressure) /
						  (2.f * pj->density) * SPIKY_GRAD * pow(H - r, 2.f);
				// compute viscosity force contribution
				fvisc +=
					VISC * MASS * (pj->vel - pi->vel) / pj->density * VISC_LAP * (H - r);
			}
		});
		vec2 fgrav = G * pi->density;
		pi->force = fpress + fvisc + fgrav;
	}
}

void EnforceBoundaryConditions(SPHParticle2D& p, const vec2& view_dim) {
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

void Integrate(SPHParticle2D* particles, int count, vec2 view_dim)
{
    for (int i = 0; i < count; ++i) {
        SPHParticle2D& p = particles[i];

        // forward Euler integration
        p.vel += DT * p.force / p.density;
        p.pos += DT * p.vel;
        EnforceBoundaryConditions(p, view_dim);
    }
}

// return cell index for a position
int pos2cell_index(vec2 pos, const SPHGrid* grid) {
    pos.x = clamp(pos.x, 0.0f, grid->dim_.x);
    pos.y = clamp(pos.y, 0.0f, grid->dim_.y);

    //vec2 cell_size = grid_dim / vec2(grid_sx, grid_sy);

    vec2 pi = pos / grid->dim_;

    // clamp because f 1.0f then id will be equal to cell_dim.x ( or y)
    int ix = clamp((int)(pi.x * (float)grid->cell_dim_.x), 0.0f, (float)grid->cell_dim_.x - 1);
    int iy = clamp((int)(pi.y * (float)grid->cell_dim_.y), 0.0f, (float)grid->cell_dim_.y - 1);

	int idx = ix + iy * grid->cell_dim_.x;
    assert(idx >= 0 && idx < grid->num_cells_);
    return idx;
}

void hash_particles(SPHGrid* grid, SPHParticle2D* particles, int num_particles, int* part_indices) {

    // classify particles to cells and vice versa
    for (int cell_idx = 0; cell_idx < grid->num_cells_; cell_idx++) {
        grid->cells_[cell_idx].part_indices_.clear();
    }

    for (int i = 0; i < num_particles; ++i) {
        SPHParticle2D& p = particles[i];

        int cell_idx = pos2cell_index(p.pos, grid);
        part_indices[i] = cell_idx;
        grid->cells_[cell_idx].addParticle(i);
    }
}

void identify_surface_cells(SPHGrid* grid, int num_particles, uint32_t* part_flags) {
    (void)num_particles;

    const int sx = grid->cell_dim_.x;
    const int sy = grid->cell_dim_.y;
    for (int y = 0; y < sy; ++y) {
        for (int x = 0; x < sx; ++x) {
            int cell_idx = x + y * sx;
            SPHGridCell& cell = grid->cells_[cell_idx];
            cell.is_surface_ = false;

            if (!cell.hasParticles()) {
                continue;
            }

			ivec2 neigh[] = {ivec2(+1, 0),	ivec2(-1, 0),  ivec2(0, +1),  ivec2(0, -1),
							 ivec2(+1, +1), ivec2(+1, -1), ivec2(-1, +1), ivec2(-1, -1)};
            
            for (int n = 0; n < (int)COUNTOF(neigh);++n) {
                int nx = x + neigh[n].x;
                int ny = y + neigh[n].y;

                if (!grid->isValidCell(nx, ny) || !grid->at(nx, ny)->hasParticles()) {
                    cell.is_surface_ = true;
                    break;
                }
            }

            if (!cell.is_surface_)
                continue;

            // this is surface cell, so mark all contained particles as surface particles
            for (int part_idx : cell.part_indices_) {
                assert(part_idx < num_particles && part_idx >= 0);
                part_flags[part_idx] = 1;
            }
        }
    }
}

// A completely parallel surface reconstruction method for particle-based fluids. Section 4.1
void identify_surface_vertices(SPHGrid* grid, SPHParticle2D* particles, int num_particles, float radius, int* part_indices, uint32_t* part_flags) {

	// check all cell in 3*radius
	const float thresholdSqr = 3 * radius * 3 * radius;
    const vec2 cells_in_radius = grid->getCellSize() / (3.0f * radius);
	const ivec2 num_cells2check = ivec2(1 + (int)cells_in_radius.x, 1 + (int)cells_in_radius.y);
    const ivec2 num_vert = grid->getNumVertices();
    const vec2 cell_size = grid->getCellSize();

    SPHGridVertex* varr = grid->vertices_[grid->cur_vert_array_];
	memset(varr, 0, sizeof(SPHGridVertex) * grid->num_vertices_);

	// classify particles to cells and vice versa
	for (int i = 0; i < num_particles; ++i) {
		// if surface particle
		if (part_flags[i]) {
            ivec2 vcoord = grid->getCellCoordFromCellIndex(part_indices[i]);
            vec2 part_pos = particles[i].pos;

            // + 2 to compensate for the fact that we get cell coord (see vcoord) which tends to be
            // "leftier", so need to check more vertices on a right side
			int n_start_x = clamp(vcoord.x - num_cells2check.x, 0, (float)num_vert.x);	
			int n_end_x = clamp(vcoord.x + num_cells2check.x + 2, 0, (float)num_vert.x);

			int n_start_y = clamp(vcoord.y - num_cells2check.y, 0, (float)num_vert.y);	
			int n_end_y = clamp(vcoord.y + num_cells2check.y + 2, 0, (float)num_vert.y);

            vec2 npos = grid->getVertexPosFromVertexCoord(n_start_x, n_start_y);
            //varr[vcoord.x + vcoord.y * num_vert.x].is_surface_ = 255;

#if 1
			for (int ny = n_start_y; ny < n_end_y; ++ny) {
			    for (int nx = n_start_x; nx < n_end_x; ++nx) {
                    vec2 cur_pos = npos + vec2((float)(nx - n_start_x)*cell_size.x, (float)(ny - n_start_y)*cell_size.y);
                    if(lengthSqr(cur_pos - part_pos) < thresholdSqr) {
                        varr[nx + ny * num_vert.x].is_surface_ = 255;
                    }
                    else {
                    // opt: if cur_pos.x > part_pos.x we on a right side and further than threshold, we can "break" to the next line
                        if(cur_pos.x > part_pos.x)
                            break;
                    }
				}
			}
#endif 
		}
	}
}

float k_func(float s_sq) {
    return  max(0.0f, (1-s_sq)*(1-s_sq)*(1-s_sq));
}

// Animating Sand as a Fluid: 5  Surface Reconstruction from Particles
void calc_sdf(SPHGrid *grid, SPHParticle2D *particles, int num_particles, float radius) {

    (void)num_particles;

	SPHGridVertex *varr = grid->vertices_[grid->cur_vert_array_];
	const int vsx = grid->cell_dim_.x + 1;
	const int vsy = grid->cell_dim_.y + 1;

    const float Rsq = 3.0f * radius * 3.0f * radius;

	for (int vy = 0; vy < vsy; vy++) {
		for (int vx = 0; vx < vsx; vx++) {
			const SPHGridVertex& vertex = varr[vy * vsx + vx];

			if (vertex.is_surface_) {

                vec2 vpos = grid->getVertexPosFromVertexCoord(vx, vy);

                float r_avg = 0;
                vec2 x_avg = vec2(0, 0);
                float denom = 0;

				// get possible particle neighbours, for this chck 4 cells around this
				// vertex
				ivec2 neigh[] = {ivec2(-1, 0), ivec2(0, -1), ivec2(-1, -1), ivec2(0, 0)};

				for (int n = 0; n < (int)COUNTOF(neigh); ++n) {
					int nx = vx + neigh[n].x;
					int ny = vy + neigh[n].y;

					if (!grid->isValidCell(nx, ny)) continue;

					SPHGridCell *cell = grid->at(nx, ny);
					for (int part_idx : cell->part_indices_) {
                        assert(part_idx < num_particles);
						vec2 ppos = particles[part_idx].pos;
    
                        float dp_sq = lengthSqr(ppos - vpos);
                        float s_sq = dp_sq/Rsq;
                        float k = k_func(s_sq);
                        if(k > 0.0f) {
                            r_avg += radius * k;
                            x_avg += ppos * k;
                            denom += k;
                        }
					}

				}

                float phi = 0.0f;
				if (denom > 0.0f) {
					r_avg = r_avg / denom;
					x_avg = x_avg / denom;
				    phi = length(vpos - x_avg) - r_avg;
				} else {
				    phi = radius;
                }
                varr[vy * vsx + vx].value_ = phi;
			}
            //printf("%.3f ", varr[vy * vsx + vx].value_);
		}
        //printf("\n");
	}
    //printf("=========\n");
}

void sph_init_renderer() {
	gos_AddRenderMaterial("deferred_sph");
	gos_AddRenderMaterial("sdf_visualize");
}

void sph_update(SPHParticle2D *particles, int count, vec2 view_dim, SPHGrid* grid) {
	ComputeDensityPressure_neigh(particles, count, grid);
	//ComputeDensityPressure(particles, count);
	ComputeForces_neigh(particles, count, grid);
	//ComputeForces(particles, count);
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
		{8, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,flags), gosVERTEX_ATTRIB_TYPE::FLOAT, 1},
	};

    static auto vdecl = gos_CreateVertexDeclaration(
        sph_vdecl, sizeof(sph_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}

extern int RendererGetNumBufferedFrames();

SPHGrid *SPHGrid::makeGrid(int sx, int sy, vec2 dim) {
	SPHGrid *grid = new SPHGrid;
	grid->dim_ = dim;
	grid->num_cells_ = sx * sy;
	grid->cell_dim_ = ivec2(sx, sy);
	grid->cells_ = new SPHGridCell[grid->num_cells_];

	grid->num_vertices_ = (sx + 1) * (sy + 1);
    const int num_vert_arrays = RendererGetNumBufferedFrames() + 1;
    grid->vertices_ = new SPHGridVertex*[num_vert_arrays];
    grid->num_vert_arrays_ = num_vert_arrays;
    grid->cur_vert_array_ = 0;
    for(int i = 0;i<num_vert_arrays;++i) {
	    grid->vertices_[i] = new SPHGridVertex[grid->num_vertices_];
    }

	return grid;
}

SPHGrid::~SPHGrid() {
	delete[] cells_;

    for(int i=0; i<num_vert_arrays_; ++i) {
	    delete[] vertices_[i];
    }
    delete[] vertices_;
}

SPHSceneObject* SPHSceneObject::Create(const vec2& view_dim, int num_particles, const vec3& pos) {
    SPHSceneObject* o = new SPHSceneObject();
    o->num_particles_ = num_particles;
    o->particles_ = new SPHParticle2D[num_particles];
    o->part_indices_ = new int[num_particles];
    o->part_flags_ = new uint32_t[num_particles];
    o->view_dim_ = view_dim;
    o->grid_ = SPHGrid::makeGrid(15, 15, view_dim);

    auto tr = o->AddComponent<TransformComponent>();
    tr->SetPosition(pos);
    o->transform_ = tr;
    return o;
}

SPHSceneObject::~SPHSceneObject() {
    DeinitRenderResources();
    delete[] particles_;
    delete[] part_indices_;
    delete[] part_flags_;
    delete grid_;
}

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
	sdf_mat_ = gos_getRenderMaterial("sdf_visualize");
    DWORD wh = ((grid_->cell_dim_.x + 1) << 16) | (grid_->cell_dim_.y + 1);
    surface_grid_tex_ = gos_NewEmptyTexture(gos_Texture_R8, "surface_grid" , wh);
    sdf_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "sdf" , wh);
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

    memset(part_flags_, 0, sizeof(int) * num_particles_);
    memset(part_indices_, 0, sizeof(int) * num_particles_);
    hash_particles(grid_, particles_, num_particles_, part_indices_);
    identify_surface_cells(grid_, num_particles_, part_flags_);
    identify_surface_vertices(grid_, particles_, num_particles_, radius_, part_indices_, part_flags_);
    calc_sdf(grid_, particles_, num_particles_, radius_);


    sph_update(particles_, num_particles_, view_dim_, grid_);

    for (int i = 0; i < num_particles_; ++i) {
        SPHParticle2D& p = particles_[i];
        p.flags = part_flags_[i] == 0 ? -1.0f : 1.0f;
    }
}

extern void render_quad(uint32_t tex_id, const vec4& scale_offset, HGOSRENDERMATERIAL pmat);

void SPHSceneObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	// update instancing buffer
	cur_inst_vb_ = (cur_inst_vb_ + 1) % ((int)inst_vb_.size());

	int num_particles = num_particles_;
	HGOSBUFFER inst_vb = inst_vb_[cur_inst_vb_];
	SPHParticle2D *particles = particles_;

    SPHGridVertex* grid_verts = grid_->vertices_[grid_->cur_vert_array_];
	grid_->cur_vert_array_ = (grid_->cur_vert_array_ + 1) % (grid_->num_vert_arrays_);
    DWORD surface_grid_tex = surface_grid_tex_;
    DWORD sdf_tex = sdf_tex_;
    ivec2 tex_wh = grid_->cell_dim_ + ivec2(1,1);

	ScheduleRenderCommand(rfc, [num_particles, inst_vb, particles, grid_verts, surface_grid_tex, sdf_tex, tex_wh]() {
		const size_t bufsize = num_particles * sizeof(SPHParticle2D);
		// TODO: think about typed buffer wrapper
		SPHParticle2D *part_data =
			(SPHParticle2D *)gos_MapBuffer(inst_vb, 0, bufsize, gosBUFFER_ACCESS::WRITE);
		memcpy(part_data, particles, bufsize);
		gos_UnmapBuffer(inst_vb);

        TEXTUREPTR texinfo;
        gos_LockTexture(surface_grid_tex, 0, false, &texinfo);
        //static_assert(sizeof(SPHGridVertex) == 1, "Sizes must be the same");
        for(int y=0; y<tex_wh.y;++y) {
            //memcpy((uint8_t*)texinfo.pTexture + y*texinfo.Pitch, grid_verts + y*tex_wh.x, tex_wh.x);
            uint8_t* row = (uint8_t*)texinfo.pTexture + y*texinfo.Pitch;
            SPHGridVertex* src_row = grid_verts + y*tex_wh.y;
            for(int x=0;x<tex_wh.x;++x) {
                row[x] = src_row[x].is_surface_ ? 255 : 0;
            }
        }
        gos_UnLockTexture(surface_grid_tex);
#if 1 
        gos_LockTexture(sdf_tex, 0, false, &texinfo);
        for(int y=0; y<tex_wh.y;++y) {
            float* row = (float*)texinfo.pTexture + y*texinfo.Pitch;
            SPHGridVertex* src_row = grid_verts + y*tex_wh.y;
            for(int x=0;x<tex_wh.x;++x) {
                row[x] = src_row[x].value_;
            }
        }
        gos_UnLockTexture(sdf_tex);
#endif
	});

    HGOSRENDERMATERIAL sdf_material = sdf_mat_;
	ScheduleDebugDrawCommand(rfc, [surface_grid_tex, sdf_tex, sdf_material]() {
		render_quad(surface_grid_tex, vec4(0.25f, 0.25f, -0.5f, 0.5f), nullptr);
		render_quad(sdf_tex, vec4(0.25f, 0.25f, 0.25f, 0.5f), sdf_material);
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
    vec2 offset = vec2(o->GetBounds().x*0.2f, 2.0f*radius);
    int row_size = (int)(o->GetBounds().x / (2.0f*radius));
    int column_size = (count + row_size - 1) / row_size; 
    for(int y= 0; y<column_size; ++y) {
        for(int x= 0; x<row_size; ++x) {
            int idx = y*row_size + x;
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
