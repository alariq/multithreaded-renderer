#pragma once

#include<vector>

class SPHSurfaceMesh {
        struct RenderMesh* mesh_ = nullptr;
        int num_vertices_ = 0;
        std::vector<struct SVD*> vb_;
        std::vector<int> vb_size_;
        int cur_vb_ = 0;
        int rt_vb_idx_ = 0;
    public:
        SPHSurfaceMesh() noexcept; 
        ~SPHSurfaceMesh() noexcept;
        void updateFromGrid(const struct SPHGrid* grid); 
        void InitRenderResources();
        void DeinitRenderResources();

        void UpdateMesh(struct RenderFrameContext *rfc) const; 
        const RenderMesh* getRenderMesh() const;
};

