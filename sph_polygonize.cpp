#include "sph_polygonize.h"
#include "sph.h"
#include "renderer.h"
#include "utils/marshingcubes.h"
#include "utils/vec.h"

extern int RendererGetNumBufferedFrames();

// probably better to change Polygonize to convert local index to grid index instead
struct MCGridAdapter {
    struct Vertex {
       vec4 pos;
       vec4 norm;
       float value;


       Vertex() = default;

       Vertex(const SPHGrid* grid, ivec2 coord, int layer) {
           const SPHGridVertex* varr = grid->vertices_[grid->cur_vert_array_];
           const SPHGridVertex& vtx = varr[coord.x + (grid->res_.x+1)*coord.y];
           value = vtx.value_;
           norm = vec4(0,0,1,0);
           vec2 p = grid->vtx2pos(coord.x, coord.y);
           pos = vec4(p.x, p.y, layer? 1.0f: 0.0f, 1.0f);
       }
    };

    struct Cell {
        static const ivec2 offset[4];
        Cell(const SPHGrid* grid, int i):grid_(grid) {
            coord_ = grid_->getCellCoordFromCellIndex(i);
        }

        Vertex p(int vert_idx) const {
            assert(vert_idx>=0 && vert_idx<8);
            int i_2d = vert_idx & 0x3;  
            return Vertex(grid_, coord_ + offset[i_2d], vert_idx&0x4);
        }
    private:
        const SPHGrid* grid_;
        ivec2 coord_;

    };

    const SPHGrid* grid_;
    Cell operator[](int i) const {
        return Cell(grid_, i);
    }
};

const ivec2 MCGridAdapter::Cell::offset[4] = { {0,0},{0,1},{1,1},{1,0} };

struct MCMeshAdapter {

    SVD* vb_;
    int vb_size_ = 0;
    int vb_capacity_ = 0;

    enum { kVertexSize = sizeof(SVD) };

    // means we are really going to use them, so size is increased as well
    void allocate_vb(int size) {
        if(vb_capacity_ < vb_size_ + size) {
            const int new_capacity = vb_capacity_ + max(size, vb_capacity_);
            SVD* new_vb = new SVD[new_capacity];
            memcpy(new_vb, vb_, kVertexSize * vb_size_);
            delete[] vb_;
            vb_ = new_vb;
            vb_capacity_ = new_capacity;
        }
        vb_size_ += size;
    }

    void p(unsigned int i, vec4 p) { vb_[i].pos = p.xyz(); }
    void n(unsigned int i, vec4 n) { vb_[i].normal = n.xyz(); }

};


SPHSurfaceMesh::SPHSurfaceMesh() noexcept {
    
    size_t num_buffers = RendererGetNumBufferedFrames() + 1;
    vb_.resize(num_buffers);
    vb_size_.resize(num_buffers);
    for(size_t i=0; i<vb_.size(); ++i) {
        vb_size_[i] = 1; 
        vb_[i] = new SVD[vb_size_[i]];
    }
}

void SPHSurfaceMesh::InitRenderResources() {
	mesh_ = new RenderMesh();
	mesh_->vdecl_ = get_svd_vdecl();
    mesh_->ib_ = 0;
    mesh_->vb_ = gos_CreateBuffer( gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW, sizeof(SVD), 1024, nullptr);
	mesh_->prim_type_ = PRIMITIVE_TRIANGLELIST;
}

void SPHSurfaceMesh::DeinitRenderResources() {
    delete mesh_;
    mesh_ = nullptr;
}

SPHSurfaceMesh::~SPHSurfaceMesh() noexcept {
    for(size_t i=0; i<vb_.size(); ++i) {
        delete[] vb_[i];
    }
}

void SPHSurfaceMesh::updateFromGrid(const SPHGrid* grid) {
    
    MCMeshAdapter mb;
    mb.vb_ = vb_[cur_vb_];
    mb.vb_capacity_ = vb_size_[cur_vb_];
    mb.vb_size_ = 0;

    MCGridAdapter mcgrid;
    mcgrid.grid_ = grid;
    
    int num_vertices = PolygoniseGrid(mcgrid, 0.0f, mb, grid->num_cells_);
    
    for(int i=0;i<num_vertices;++i) {
        mb.vb_[i].uv.y = mb.vb_[i].pos.z;
        mb.vb_[i].uv.x = (mb.vb_[i].pos.x - grid->domain_min_.x) / (grid->domain_max_.x - grid->domain_min_.x);
    }

    // because it can be reallocated have to assign back
    vb_[cur_vb_] = mb.vb_;
    vb_size_[cur_vb_] = mb.vb_size_;

    rt_vb_idx_ = cur_vb_;

    size_t num_buffers = RendererGetNumBufferedFrames() + 1;
    cur_vb_ = (cur_vb_ + 1) % num_buffers;
}

void SPHSurfaceMesh::UpdateMesh(struct RenderFrameContext *rfc) const {

	SVD *vb = vb_[rt_vb_idx_];
	int vb_size = vb_size_[rt_vb_idx_];
	RenderMesh *mesh = mesh_;

    mesh_->vb_count_ = vb_size;

	ScheduleRenderCommand(rfc, [mesh, vb, vb_size]() {
		size_t num_bytes = mesh->vb_ ? gos_GetBufferSizeBytes(mesh->vb_) : 0;
        size_t new_num_bytes = vb_size * sizeof(SVD);
		if (num_bytes < new_num_bytes) {
            gos_ResizeBuffer(mesh->vb_, vb_size);
		}
        gos_UpdateBuffer(mesh->vb_, vb, 0, vb_size * sizeof(SVD));
	});
}

const RenderMesh* SPHSurfaceMesh::getRenderMesh() const {
    return mesh_;
}
