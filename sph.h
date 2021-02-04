#pragma once

#include "utils/vec.h"
#include "sph_kernels.h"
#include "sph_boundary.h"
#include <cassert>
#include <vector>

int pos2idx(const vec3& pos, const ivec3& res, const vec3& domain_min, const vec3& domain_max);
vec3 idx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max);
vec3 vtx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max);

enum SPHParticleFlags : uint32_t {
    kSPHFlagActive = 0x01,
    kSPHFlagBoundary = 0x02,
    kSPHFlagSurface = 0x04
};

struct SPHParticle2D {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
    uint32_t flags;
};

typedef SPHParticle2D SPHInstVDecl ;


struct SPHGridCell {
    // particles which fall into this cell
    std::vector<int> part_indices_;
    bool is_surface_;

    bool hasParticles() { return 0 != part_indices_.size(); }
    void addParticle(int part_index) { part_indices_.push_back(part_index); }
};

struct SPHGridVertex {
    uint8_t is_surface_; 
    // signed distance function value 
    float value_;
    vec2 pos_avg;
};

struct SPHGrid {
    SPHGridCell* cells_;
    int num_cells_;

    SPHGridVertex** vertices_;
    int num_vert_arrays_;
    int cur_vert_array_;
    int cur_vert_array_rt_;
    int num_vertices_;

    ivec2 res_;
    vec2 domain_max_;
    vec2 domain_min_;

    SPHGridCell* at(int x, int y) {
        assert(x >= 0 && x < res_.x);
        assert(y >= 0 && y < res_.y);
        return &cells_[x + y * res_.x];
    }

    bool isValidCell(int x, int y) const {
        if (x >= 0 && x < res_.x && y >= 0 && y < res_.y)
            return true;
        return false;
    }

    // linearIndex()
    ivec2 getCellCoordFromCellIndex(int idx) const  {
		ivec2 coord = ivec2(idx % res_.x, idx / res_.y);
        return coord;
	}

    ivec2 getNumVertices() const {
        return ivec2(res_.x, res_.y);
    }

    int vertexDimX() { return res_.x + 1; }
    int vertexDimY() { return res_.y + 1; }

    vec2 getCellSize() const {
        vec2 cell_count = vec2((float)res_.x, (float)res_.y);
        return (domain_max_-domain_min_)/cell_count;
    }

    int pos2idx(const vec2 &pos) const {
		return ::pos2idx(vec3(pos.x, pos.y, 0.0f), ivec3(res_.x, res_.y, 1),
						 vec3(domain_min_.x, domain_min_.y, 0.0f),
						 vec3(domain_max_.x, domain_max_.y, 1.0f));
	}

	vec2 idx2pos(int x, int y) const {
		return ::idx2pos(ivec3(x, y, 0), ivec3(res_.x, res_.y, 1),
						 vec3(domain_min_.x, domain_min_.y, 0.0f),
						 vec3(domain_max_.x, domain_max_.y, 1.0f)).xy();
	}

	vec2 vtx2pos(int x, int y) const {
		return ::vtx2pos(ivec3(x, y, 0), ivec3(res_.x+1, res_.y+1, 1),
						 vec3(domain_min_.x, domain_min_.y, 0.0f),
						 vec3(domain_max_.x, domain_max_.y, 1.0f)).xy();
	}


    static SPHGrid* makeGrid(int sx, int sy, vec2 dim);
    ~SPHGrid();
};

struct SPHFluidModel {
    SPHGrid* grid_;
    std::vector<SPHParticle2D> particles_;

    float radius_;
    float density0_;
    float volume_;
    float support_radius_;

    // return index of the first added particle
	int add(size_t count) {
        auto old_size = particles_.size();
		particles_.resize(old_size + count);
        for (int i = 0; i < (int)count; i++) {
            particles_[old_size + i].flags = 0;
        }
        return (int)old_size;
	}
};

struct SPHStateData {
    std::vector<uint32_t> flags_;
    std::vector<int> cell_indices_;

	void allocate(size_t count) {
		flags_.resize(count);
		cell_indices_.resize(count);
	}
    void clear_data() {
		memset(flags_.data(), 0, sizeof(int) * flags_.size());
		memset(cell_indices_.data(), 0, sizeof(int) * cell_indices_.size());
    }
};

struct SPHSimData {
    std::vector<float> kappa_;
    std::vector<float> factor_;
    std::vector<float> density_adv_;

    // from boundary (overkill because only a fraction of particles needs this dunring a simulation frame)
    // use index into another array (resized on demand, or using fixed block allocator scheme)
    std::vector<vec2> boundaryXj_;
    std::vector<float> boundaryVolume_;

	void allocate(size_t count) {
		density_adv_.resize(count);
		factor_.resize(count);
		kappa_.resize(count);
		boundaryXj_.resize(count);
		boundaryVolume_.resize(count);
	}
};


class SPHSimulation {

    // make solvers friends (for now)
    friend struct DF_TimeStep;
    friend void SPH_OldTick(SPHSimulation* sim);
    friend void sph_update();

    const vec2 accel_ = vec2(0.0f, -9.81f);
    const float radius_ = 0.1f;
    const float support_radius_ = 4.0f * radius_;

    float time_step_ = 0.016f;
    float cfl_factor_ = 0.5f;
    float min_cfl_timestep_ = 0.0001f;
    float max_cfl_timestep_ = 0.016f;//0.005;

    float W_zero_;
	float (*kernel_fptr_)(const vec3 &);
	vec3 (*grad_kernel_fptr_)(const vec3& r);

public:
    SPHSimulation() = default;

    SPHSimulation(const SPHSimulation&) = delete;
    SPHSimulation(SPHSimulation&&) = delete;

    ~SPHSimulation() {
        delete sim_data_;
        delete state_data_;
    }

    void Setup() {
	    CubicKernel::setRadius(support_radius_);
	    CubicKernel2D::setRadius(support_radius_);

        kernel_fptr_ = CubicKernel2D::W;
        grad_kernel_fptr_ = CubicKernel2D::gradW;
        W_zero_ = CubicKernel2D::W_zero();
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

    float Wzero() { return W_zero_; }

    void updateTimestep() {
        SPHFluidModel* fm = fluid_model_;
        float max_vel = 0.01f;
        const int num_particles = (int)fm->particles_.size();
		for (int i = 0; i < num_particles; ++i) {
			SPHParticle2D &pi = fm->particles_[i];
            float vel = lengthSqr(pi.vel + accel_*time_step_);
            max_vel = max(max_vel, vel);
        }
        
        float diameter = fm->radius_ * 2.0f;
        float h = cfl_factor_ * diameter / sqrtf(max_vel);
        h = clamp(h, min_cfl_timestep_, max_cfl_timestep_);
        time_step_ = h;
    }
    void setFluid(struct SPHFluidModel*);
    void setBoundary(class SPHBoundaryModel*);

private:
    struct SPHFluidModel* fluid_model_ = nullptr;
    struct SPHSimData* sim_data_ = nullptr;
    struct SPHStateData* state_data_ = nullptr;

    class SPHBoundaryModel* boundary_model_ = nullptr;

};

void sph_init();
void sph_init_renderer();
void sph_deinit();
void sph_update();
SPHSimulation* sph_get_simulation();

// surface identification
void hash_particles(SPHGrid* grid, SPHParticle2D* particles, int num_particles, int* part_cell_indices) ;
void identify_surface_cells(SPHGrid* grid, int num_particles, uint32_t* part_flags) ;
void identify_surface_vertices(SPHGrid* grid, SPHParticle2D* particles, int num_particles, float radius, int* part_cell_indices, const uint32_t* part_flags) ;
void calc_sdf(SPHGrid *grid, const SPHParticle2D *particles, int num_particles, float radius) ;
//

// Does not include particle i itself!
void foreach_in_radius(float r, int i, SPHGrid *grid, SPHParticle2D *particles,
					   int num_particles,
					   std::function<void(const SPHParticle2D *, int)> func);

