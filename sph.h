#pragma once

#include "obj_model.h"
#include "utils/vec.h"
#include <atomic>

struct SPHParticle2D {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
    float flags;
};

typedef SPHParticle2D SPHInstVDecl ;

void sph_init();
void sph_deinit();
void sph_update(SPHParticle2D *particles, int count, vec2 view_dim);

int pos2idx(const vec3& pos, const ivec3& res, const vec3& domain_min, const vec3& domain_max);
vec3 idx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max);
vec3 vtx2pos(const ivec3& idx, const ivec3 res, const vec3 domain_min, const vec3& domain_max);

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

class SPHSceneObject : public GameObject {
    SPHParticle2D* particles_;
    uint32_t* part_flags_;
    int* part_indices_;
    int num_particles_;
    vec2 view_dim_;
    float radius_;
    SPHGrid* grid_;
    class SPHSurfaceMesh* surface_;
    // cached transform component
    TransformComponent* transform_ = nullptr;

    RenderMesh* sphere_mesh_ = nullptr; 
    HGOSVERTEXDECLARATION vdecl_;
    std::vector<HGOSBUFFER> inst_vb_;
    mutable int cur_inst_vb_ = 0;
    
    HGOSRENDERMATERIAL  mat_;
    HGOSRENDERMATERIAL  sdf_mat_;
    DWORD surface_grid_tex_;
    DWORD sdf_tex_;

    std::atomic_bool b_initalized_rendering_resources;

public:
    static SPHSceneObject* Create(const vec2& view_dim, int num_particles, const vec3& pos);

    virtual void Update(float /*dt*/) override;

    virtual const char* GetName() const override { return "SPH Scene"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
    virtual ~SPHSceneObject();

	float GetRadius() { return radius_; }

    const vec2& GetBounds() { return view_dim_; }

	SPHParticle2D* GetParticles() { return particles_; }
	int GetParticlesCount() { return num_particles_; }

    virtual void InitRenderResources() override;
    void DeinitRenderResources();

    // IRenderable
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

void initialize_particle_positions(SPHSceneObject* o);
void sph_init_renderer();


// Does not include particle i itself!
void foreach_in_radius(float r, int i, SPHGrid *grid, SPHParticle2D *particles,
					   int num_particles,
					   std::function<void(const SPHParticle2D *, int)> func);
