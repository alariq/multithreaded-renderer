#include "sph.h"
#include "engine/utils/kernels.h"
#include "sph_solver_df.h"
#include "sph_solver_pbd.h"
#include "sph_boundary.h"
#include "sph_emitter.h"
#include "sph_polygonize.h"
#include <cmath>
#include "obj_model.h"
#include "res_man.h"
#include "sph_editor.h"
#include "engine/gameos.hpp" // COUNTOF
#include "utils/vec.h"
#include <functional>
#include <cfloat>

static SPHSimulation* g_sim = nullptr;
static SPHEmitterSystem* g_emitter_system = nullptr;
void sph_init() {
    assert(nullptr == g_sim);
    g_sim = new SPHSimulation();
    g_sim->Setup();
    g_sim->setSolver(Get_SPH_PBD_SolverInterface());
    g_emitter_system = new SPHEmitterSystem();
    sph_editor_init();
}

void sph_init_renderer() {
	gos_AddRenderMaterial("deferred_sph");
	gos_AddRenderMaterial("sdf_visualize");
}

void sph_deinit() {
    assert(nullptr != g_sim);
    sph_editor_deinit();

    delete g_emitter_system;
    g_emitter_system = nullptr;

    delete g_sim;
    g_sim = nullptr;
}

SPHEmitterSystem* sph_get_emitter_system() {
    return g_emitter_system;
}

bool g_calc_sdf = false;
void sph_update(float dt) {

    g_emitter_system->step(dt);

	{
		SPHFluidModel *fm = g_sim->fluid_model_;
		for (SPHParticle2D &p : fm->particles_) {
			if (p.flags & kSPHFlagPending) {
				g_sim->addToSimulation(&p, fm);
				p.flags &= ~kSPHFlagPending;
				p.flags |= kSPHFlagActive;
			}
		}
	}

	// simulate particle movement
    g_sim->Tick(dt);

    if(gos_GetKeyStatus(KEY_T) == KEY_PRESSED)
        g_calc_sdf = !g_calc_sdf;

    // calculate surface
    g_sim->state_data_->clear_data();
    SPHFluidModel* fm = g_sim->fluid_model_;
    int32_t* cell_indices = g_sim->state_data_->cell_indices_.data();
    uint32_t* part_flags = g_sim->state_data_->flags_.data();
    const int num_particles = (int)fm->particles_.size();

    hash_particles(fm->grid_, fm->particles_.data(), num_particles, cell_indices);
    identify_surface_cells(fm->grid_, num_particles, part_flags);
    identify_surface_vertices(fm->grid_, fm->particles_.data(), num_particles, fm->radius_, cell_indices, part_flags);
    if(g_calc_sdf)
        calc_sdf(fm->grid_, fm->particles_.data(), num_particles, fm->radius_);

    for (int i = 0; i < num_particles; ++i) {
        SPHParticle2D& p = fm->particles_[i];
        p.flags = part_flags[i] == 0 ? 0 : kSPHFlagSurface; 
    }
}

SPHSimulation* sph_get_simulation() {
    gosASSERT(g_sim);
    return g_sim;
}

int pos2idx(const vec3& pos, const ivec3& res, const vec3& domain_min, const vec3& domain_max)
{
    static const float my_eps = 1.0e+31F*FLT_MIN;

    vec3 p = (pos - domain_min) / (domain_max - domain_min) ;
    // -eps to avoid getting index which equals to resolution
    p.x = clamp(p.x, 0.0f, 1.0f - my_eps);
    p.y = clamp(p.y, 0.0f, 1.0f - my_eps);
    p.z = clamp(p.z, 0.0f, 1.0f - my_eps);

    vec3 fres = vec3((float)res.x, (float)res.y, (float)res.z);
    vec3 fi = fres * p;
    ivec3 i = ivec3((int)fi.x, (int)fi.y, (int)fi.z);
    assert(i.x >= 0 && i.x < res.x);
    assert(i.y >= 0 && i.y < res.y);
    assert(i.z >= 0 && i.z < res.z);

	int idx = i.x + (i.y + i.z * res.y) * res.x;
    assert(idx >= 0 && idx < res.x*res.y*res.z);
    return idx;
}
// for the case when values are stored at the cell centers (i.e. as in textyres)
vec3 idx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max) {
    assert(idx.x >= 0 && idx.x < res.x);
    assert(idx.y >= 0 && idx.y < res.y);
    assert(idx.z >= 0 && idx.z < res.z);

    vec3 p = vec3((float)idx.x, (float)idx.y, (float)idx.z);
    p += vec3(0.5f);
    p /= vec3((float)res.x, (float)res.y, max(1.0f, (float)res.z - 1.0f));
    return p * (domain_max - domain_min) + domain_min;
}

// for the case when values are stored at the cell corners (i.e. grid of vertices)
// [0, res-1] -> [domain_min, domain_max]
vec3 vtx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max) {
    assert(idx.x >= 0 && idx.x < res.x);
    assert(idx.y >= 0 && idx.y < res.y);
    assert(idx.z >= 0 && idx.z < res.z);

    vec3 p = vec3((float)idx.x, (float)idx.y, (float)idx.z);
    //p += vec3(0.5f);
    p /= vec3((float)res.x - 1, (float)res.y - 1, max(1.0f, (float)res.z - 1.0f));
    return p * (domain_max - domain_min) + domain_min;
}

// Does not include particle i itself!
void foreach_in_radius(float r, int idx, SPHGrid *grid, SPHParticle2D *particles,
					   int num_particles,
					   std::function<void(const SPHParticle2D *, int)> func) {

	(void)num_particles;
	const vec2 &pos = particles[idx].pos;

	const int left_top = grid->pos2idx(pos - vec2(r, r));
	const int right_bottom = grid->pos2idx(pos + vec2(r, r));
	const float r_sq = r * r;
	const ivec2 lt_coord = grid->getCellCoordFromCellIndex(left_top);
	const ivec2 rb_coord = grid->getCellCoordFromCellIndex(right_bottom);

	int n_start_x = max(lt_coord.x - 1, 0);
	int n_end_x = min(rb_coord.x + 1, grid->res_.x - 1);

	int n_start_y = max(lt_coord.y - 1, 0);
	int n_end_y = min(rb_coord.y + 1, grid->res_.y - 1);

	for (int ny = n_start_y; ny <= n_end_y; ++ny) {
		for (int nx = n_start_x; nx <= n_end_x; ++nx) {
			const SPHGridCell *cell = grid->at(nx, ny);
			for (int pj_idx : cell->part_indices_) {
				assert(pj_idx < num_particles);
				if (idx != pj_idx && lengthSqr(pos - particles[pj_idx].pos) <= r_sq)
					func(&particles[pj_idx], pj_idx);
			}
		}
	}
}

void hash_particles(SPHGrid* grid, SPHParticle2D* particles, int num_particles, int* part_cell_indices) {

    // classify particles to cells and vice versa
    for (int cell_idx = 0; cell_idx < grid->num_cells_; cell_idx++) {
        grid->cells_[cell_idx].part_indices_.clear();
    }

    for (int i = 0; i < num_particles; ++i) {
        SPHParticle2D& p = particles[i];

        int cell_idx = grid->pos2idx(p.pos);
        part_cell_indices[i] = cell_idx;
        grid->cells_[cell_idx].addParticle(i);
    }
}

void identify_surface_cells(SPHGrid* grid, int num_particles, uint32_t* part_flags) {
    (void)num_particles;

    const int sx = grid->res_.x;
    const int sy = grid->res_.y;
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
void identify_surface_vertices(SPHGrid* grid, SPHParticle2D* particles, int num_particles, float radius, int* part_indices, const uint32_t* part_flags) {

	// check all cell in 3*radius
	const float thresholdSqr = 3 * radius * 3 * radius;
    const vec2 cells_in_radius = vec2(3.0f * radius) / grid->getCellSize() ;
	const ivec2 num_cells2check = ivec2(1 + (int)cells_in_radius.x, 1 + (int)cells_in_radius.y);
    const ivec2 num_vert = grid->getNumVertices();
    const vec2 cell_size = grid->getCellSize();

    SPHGridVertex* varr = grid->vertices_[grid->cur_vert_array_];
	memset(varr, 255, sizeof(SPHGridVertex) * grid->num_vertices_);

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

            vec2 npos = grid->idx2pos(n_start_x, n_start_y);
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
void calc_sdf(SPHGrid *grid, const SPHParticle2D *particles, int num_particles, float radius) {

    (void)num_particles;

	SPHGridVertex *varr = grid->vertices_[grid->cur_vert_array_];
	const int vsx = grid->vertexDimX();
	const int vsy = grid->vertexDimY();

    const float oo_Rsq = 1.0f / (3.0f * radius * 3.0f * radius);
    vec2 search_range = vec2(3.0f * radius) / grid->getCellSize();
    ivec2 num_cells2check = ivec2((int)search_range.x  + 1, (int)search_range.y + 1);
    const ivec2 num_vert = grid->getNumVertices();

	for (int vy = 0; vy < vsy; vy++) {
		for (int vx = 0; vx < vsx; vx++) {
			const SPHGridVertex& vertex = varr[vy * vsx + vx];

			if (vertex.is_surface_) {

                vec2 vpos = grid->vtx2pos(vx, vy);

                float r_avg = 0;
                vec2 x_avg = vec2(0, 0);
                float denom = 0;

                // TODO: this has to be rewritten by just querying for neighbours in a radius
                // otherwise too many cells have to be visited if cell size if small

                int n_start_x = clamp(vx - num_cells2check.x, 0, (float)num_vert.x);	
			    int n_end_x = clamp(vx + num_cells2check.x + 2, 0, (float)num_vert.x);

			    int n_start_y = clamp(vy - num_cells2check.y, 0, (float)num_vert.y);	
			    int n_end_y = clamp(vy + num_cells2check.y + 2, 0, (float)num_vert.y);

				for (int ny = n_start_y; ny < n_end_y; ++ny) {
					for (int nx = n_start_x; nx < n_end_x; ++nx) {

						if (!grid->isValidCell(nx, ny)) continue;

						SPHGridCell *cell = grid->at(nx, ny);
						for (int part_idx : cell->part_indices_) {
							assert(part_idx < num_particles);
							vec2 ppos = particles[part_idx].pos;

							float dp_sq = lengthSqr(ppos - vpos);
							float k = k_func(dp_sq * oo_Rsq);
							if (k > 0.0f) {
								r_avg += radius * k;
								x_avg += ppos * k;
								denom += k;
							}
						}
					}
				}

				float phi = 0.0f;
				if (denom > 0.0f) {
					r_avg = r_avg / denom;
					x_avg = x_avg / denom;
				    phi = length(vpos - x_avg) - r_avg;
				} else {
                    x_avg = vec2(0.0f);
				    phi = 9999.0f;
                }
                varr[vy * vsx + vx].value_ = phi;
                varr[vy * vsx + vx].pos_avg = x_avg;
			}
            //printf("%.3f ", varr[vy * vsx + vx].value_);
		}
        //printf("\n");
	}
    //printf("=========\n");
#if 1    
    float t_low = 0.4f;
    float t_high = 3.5f;
   
    // A Unified Particle Model for Fluid-Solid Interactions. Surface Reconstruction
	for (int vy = 0; vy < vsy; vy++) {
		for (int vx = 0; vx < vsx; vx++) {
			SPHGridVertex& vertex = varr[vy * vsx + vx];

            if(vertex.value_ == 9999.0f)
                continue;

            int idx10 = vy * vsx + min(vx + 1, vsx-1);
            int idx01 = min(vy + 1, vsy-1) * vsx + vx;
            vec2 x10 = varr[idx10].pos_avg;
            vec2 x01 = varr[idx01].pos_avg;

            vec2 x_avg = vertex.pos_avg;
            vec2 dpdx = (x10 - x_avg) / grid->getCellSize().x;
            vec2 dpdy = (x01 - x_avg) / grid->getCellSize().y;

			mat2 grad_x_avg = mat2(dpdx.x, dpdy.x, dpdx.y, dpdy.y);
			mat2 m = grad_x_avg;
			vec2 coeff = vec2(1.0f, -1.0f);
			vec2 eigen = vec2(0.5f * (m.e00 + m.e11)) +
						  coeff * sqrtf(4 * m.e01 * m.e10 + sqr(m.e00 - m.e11));

            float e_max = max(eigen.x, eigen.y);
            e_max = min(e_max, t_high);
            float gamma = (t_high - e_max)/(t_high - t_low);

            float f = e_max < t_low ? 1.0f : gamma*gamma*gamma - 3.0f*gamma*gamma + 3.0f*gamma;

            vec2 vpos = grid->vtx2pos(vx, vy);
			float phi = length(vpos - x_avg) - radius * f;
			vertex.value_ = phi; 
		}
	}
#endif
}

void SPHSimulation::setFluid(struct SPHFluidModel* fm) {
    // replace the current fluid (only one at a time supported at the moment)

    fluid_model_ = fm;
    if(!fm->sim_data_) {
        fm->sim_data_ = solver_->CreateSimData();
        fm->sim_data_->allocate(fm->particles_.size());
    }

    if (!state_data_) {
        state_data_ = new SPHStateData();
        state_data_->allocate(fm->particles_.size());
    }
}

// you have not seen that, I have mot done that
void SPHSimulation::addToSimulation(const SPHParticle2D* p, struct SPHFluidModel* fm) {
    int idx = fm->sim_data_->append(1);
    fm->sim_data_->init(p, idx);
    state_data_->append(1);
}

void SPHSimulation::addBoundary(SPHBoundaryModel* boundary) {
    boundary_models_.push_back(boundary);
}

void SPHSimulation::removeBoundary(const SPHBoundaryModel* boundary) {
    auto b = std::begin(boundary_models_);
    auto e = std::end(boundary_models_);
    boundary_models_.erase(std::remove(b, e, boundary), e);
}


// Does not include particle i itself!
void foreach_in_radius(float r, int i, SPHGrid *grid, SPHParticle2D *particles,
					   int num_particles,
					   std::function<void(const SPHParticle2D *, int)> func);

extern int RendererGetNumBufferedFrames();

SPHGrid *SPHGrid::makeGrid(int sx, int sy, vec2 dim) {
	SPHGrid *grid = new SPHGrid;
    grid->domain_min_ = vec2(0, 0);
    grid->domain_max_ = dim;
	grid->num_cells_ = sx * sy;
	grid->res_ = ivec2(sx, sy);
	grid->cells_ = new SPHGridCell[grid->num_cells_];

    uint32_t vsx = grid->vertexDimX();
    uint32_t vsy = grid->vertexDimY();
	grid->num_vertices_ = vsx*vsy;
    const int num_vert_arrays = RendererGetNumBufferedFrames() + 1;
    grid->vertices_ = new SPHGridVertex*[num_vert_arrays];
    grid->num_vert_arrays_ = num_vert_arrays;
    grid->cur_vert_array_ = 0;
    grid->cur_vert_array_rt_ = 0;
    for(int i = 0;i<num_vert_arrays;++i) {
	    grid->vertices_[i] = new SPHGridVertex[grid->num_vertices_];
        for(uint32_t y=0;y<vsy;++y) {
            for(uint32_t x=0;x<vsx;++x) {
				grid->vertices_[i][y * vsx + x].pos_avg = grid->vtx2pos(x, y);
			}
        }
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

