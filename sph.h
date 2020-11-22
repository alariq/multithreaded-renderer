#pragma once

#include "obj_model.h"
#include "utils/vec.h"

struct SPHParticle2D {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
    float flags;
};

typedef SPHParticle2D SPHInstVDecl ;

void sph_update(SPHParticle2D *particles, int count, vec2 view_dim);

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
};

struct SPHGrid {
    SPHGridCell* cells_;
    int num_cells_;

    SPHGridVertex** vertices_;
    int num_vert_arrays_;
    int cur_vert_array_;
    int num_vertices_;

    ivec2 cell_dim_;
    vec2 dim_;

    SPHGridCell* at(int x, int y) {
        assert(x >= 0 && x < cell_dim_.x);
        assert(y >= 0 && y < cell_dim_.y);
        return &cells_[x + y * cell_dim_.x];
    }

    bool isValidCell(int x, int y) const {
        if (x >= 0 && x < cell_dim_.x && y >= 0 && y < cell_dim_.y)
            return true;
        return false;
    }

    ivec2 getCellCoordFromCellIndex(int idx) const  {
		ivec2 coord = ivec2(idx % cell_dim_.x, idx / cell_dim_.y);
        return coord;
	}

    ivec2 getVetexCoordFromVertexIndex(int idx) const  {
		ivec2 coord = ivec2(idx % (cell_dim_.x + 1), idx / (cell_dim_.y + 1));
        return coord;
	}

    vec2 getVertexPosFromVertexCoord(int x, int y) const {
		vec2 normalized = vec2((float)x / ((float)cell_dim_.x + 1.0f),
							   (float)y / ((float)cell_dim_.y + 1.0f));
		return normalized * dim_;
    }

    ivec2 getNumVertices() const {
        return ivec2(cell_dim_.x+1, cell_dim_.y+1);
    }

    vec2 getCellSize() const {
        vec2 cell_count = vec2((float)cell_dim_.x, (float)cell_dim_.y);
        return dim_/cell_count;
    }

    static SPHGrid* makeGrid(int sx, int sy, vec2 dim);
    ~SPHGrid();
};

int pos2cell_index(vec2 pos, const SPHGrid* grid);


class SPHSceneObject : public GameObject {
    SPHParticle2D* particles_;
    uint32_t* part_flags_;
    int* part_indices_;
    int num_particles_;
    vec2 view_dim_;
    float radius_;
    SPHGrid* grid_;
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

public:
    static SPHSceneObject* Create(const vec2& view_dim, int num_particles, const vec3& pos);

    virtual void Update(float /*dt*/) override;

    virtual const char* GetName() const override { return "SPH Scene"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
    virtual ~SPHSceneObject();

	void SetRadius(float r) { radius_ = r; }
	float GetRadius() { return radius_; }

    const vec2& GetBounds() { return view_dim_; }

	SPHParticle2D* GetParticles() { return particles_; }
	int GetParticlesCount() { return num_particles_; }

    virtual void InitRenderResources() override;
    void DeinitRenderResources();

    // Renderable
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

void initialize_particle_positions(SPHSceneObject* o);
void sph_init_renderer();

void foreach_in_radius(float r, vec2 pos, SPHGrid *grid, SPHParticle2D *particles,
					   int num_particles,
					   std::function<void(const SPHParticle2D *, int)> func);
