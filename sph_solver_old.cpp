#include "sph_solver_old.h"
#include "sph.h"
#include "sph_boundary.h"

#include "utils/vec.h"

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

void ComputeDensityPressure_neigh(SPHParticle2D *particles, int count, SPHGrid *grid) {
	for (int i = 0; i < count; ++i) {
		SPHParticle2D &pi = particles[i];
        pi.density = MASS * POLY6 * pow(HSQ, 3.f); // my own
		foreach_in_radius(H, i, grid, particles, count, [&pi](const SPHParticle2D* pj, int) {
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

		foreach_in_radius(H, i, grid, particles, count, [pi, &fpress, &fvisc](const SPHParticle2D* pj, int) {

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

void EnforceBoundaryConditions(SPHParticle2D& p, const vec3& view_dim) {
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
        EnforceBoundaryConditions(p, vec3(view_dim.x, view_dim.y, 0.0f));
    }
}

void SPH_OldTick(SPHSimulation* sim) {
	SPHParticle2D* particles = sim->fluid_model_->particles_.data();
	const int count = (int)sim->fluid_model_->particles_.size();
	SPHGrid* grid = sim->fluid_model_->grid_;
	vec3 dim = sim->boundary_models_[0]->getDimension();

	ComputeDensityPressure_neigh(particles, count, grid);
	ComputeForces_neigh(particles, count, grid);
	Integrate(particles, count, dim.xy());
}

