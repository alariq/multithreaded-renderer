#include "sph_object.h"
#include "sph.h"
#include "sph_boundary.h"
#include "sph_polygonize.h"
#include "res_man.h"
#include "utils/vec.h"

void initialize_particle_positions(SPHFluidModel* fm);

SPHSceneObject* SPHSceneObject::Create(const vec2& view_dim, int num_particles, const vec3& pos) {
    SPHSceneObject* o = new SPHSceneObject();
    o->view_dim_ = view_dim;
    o->radius_ = 0.1f;
	// recommended by 2014_EG_SPH_STAR.pdf 7.1
	// float grid_cell = o->radius_;
	// vec2 grid_res = view_dim / grid_cell;
	//    ivec2 igrid_res = ivec2(min(grid_res.x, 64.0f), min(grid_res.y, 64.0f));
	//   o->grid_ = SPHGrid::makeGrid(igrid_res.x, igrid_res.y, view_dim);

    const bool b_is2d = true;

	o->grid_ = SPHGrid::makeGrid(64, 64, view_dim);
    auto tr = o->AddComponent<TransformComponent>();
    tr->SetPosition(pos);
    o->transform_ = tr;
    o->b_initalized_rendering_resources = false;

    SPHFluidModel* fm = new SPHFluidModel();
    fm->density0_ = 1000;
    fm->radius_ = o->radius_;
    fm->support_radius_ = 4.0f * fm->radius_;
    // slightly reduced square (cube for 3D)
    const float diameter = 2.0f * fm->radius_;
    if (b_is2d) {
        fm->volume_ = diameter * diameter * 0.8f;
    } else {
        fm->volume_ = diameter * diameter * diameter * 0.8f;
    }
	fm->particles_.resize(num_particles);
    fm->grid_ = o->grid_;
    initialize_particle_positions(fm);

    SPHBoundaryModel* bm = new SPHBoundaryModel();
    float volume_map_cell_size = 0.1f;
    ivec3 resolution = ivec3((int)(o->view_dim_.x / volume_map_cell_size), (int)(o->view_dim_.y / volume_map_cell_size), 1);
    bm->Initialize(vec3(o->view_dim_.x, o->view_dim_.y, 10000.0f), fm->radius_, fm->support_radius_, resolution, b_is2d);

    o->fluid_ = fm;
    o->boundary_ = bm;
    o->surface_ = new SPHSurfaceMesh();
    
    SPHSimulation* sim = sph_get_simulation();
    sim->setBoundary(o->boundary_);
    sim->setFluid(o->fluid_);

	return o;
}

SPHSceneObject::~SPHSceneObject() {
    DeinitRenderResources();
    delete surface_;
    delete[] part_indices_;
    delete[] part_flags_;
    delete grid_;
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
		{8, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,flags), gosVERTEX_ATTRIB_TYPE::UNSIGNED_INT, 1},
	};

    static auto vdecl = gos_CreateVertexDeclaration(
        sph_vdecl, sizeof(sph_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}


extern int RendererGetNumBufferedFrames();
void SPHSceneObject::InitRenderResources() {
	sphere_mesh_ = res_man_load_mesh("sphere");

    int num_buffers = RendererGetNumBufferedFrames() + 1;
    inst_vb_.resize(num_buffers);
	for (int32_t i = 0; i < num_buffers; ++i) {
		inst_vb_[i] =
			gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
							 sizeof(SPHInstVDecl), (uint32_t)fluid_->particles_.size(), nullptr);
	}

    vdecl_ = get_sph_vdecl();
	mat_ = gos_getRenderMaterial("deferred_sph");
	sdf_mat_ = gos_getRenderMaterial("sdf_visualize");
    surface_grid_tex_ = gos_NewEmptyTexture(gos_Texture_R8, "surface_grid", grid_->res_.x, grid_->res_.y);
    sdf_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "sdf" , grid_->vertexDimX(), grid_->vertexDimY());

    boundary_->InitializeRenderResources();
    surface_->InitRenderResources();

    b_initalized_rendering_resources = true;

}

void SPHSceneObject::DeinitRenderResources() {
    b_initalized_rendering_resources = false;

    surface_->DeinitRenderResources();

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

    surface_->updateFromGrid(grid_);

    grid_->cur_vert_array_rt_ = grid_->cur_vert_array_;
	grid_->cur_vert_array_ = (grid_->cur_vert_array_ + 1) % (grid_->num_vert_arrays_);
}

extern void render_quad(uint32_t tex_id, const vec4& scale_offset, HGOSRENDERMATERIAL pmat);

void SPHSceneObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	if (!b_initalized_rendering_resources) return;

	// update instancing buffer
	cur_inst_vb_ = (cur_inst_vb_ + 1) % ((int)inst_vb_.size());

	const int num_particles = (int)fluid_->particles_.size();
	HGOSBUFFER inst_vb = inst_vb_[cur_inst_vb_];
	const SPHParticle2D *particles = fluid_->particles_.data();

    SPHGridVertex* grid_verts = grid_->vertices_[grid_->cur_vert_array_rt_];
    DWORD surface_grid_tex = surface_grid_tex_;
    DWORD sdf_tex = sdf_tex_;
    uint32_t vsx = grid_->vertexDimX();
    uint32_t vsy = grid_->vertexDimY();
    SPHGrid* grid = grid_;
    SPHBoundaryModel* boundary = boundary_;

	ScheduleRenderCommand(rfc, [num_particles, inst_vb, particles, grid_verts, grid, boundary, surface_grid_tex, sdf_tex, vsx, vsy]() {
		const size_t bufsize = num_particles * sizeof(SPHParticle2D);
		// TODO: think about typed buffer wrapper
		SPHParticle2D *part_data =
			(SPHParticle2D *)gos_MapBuffer(inst_vb, 0, bufsize, gosBUFFER_ACCESS::WRITE);
		memcpy(part_data, particles, bufsize);
		gos_UnmapBuffer(inst_vb);
#ifdef DEBUG_DRAW_PARTICLE_OUTLINE
        for(int i=0; i<num_particles;++i) {
            vec3 p = vec3(particles[i].pos.x, particles[i].pos.y, 0);
            //ceneter
            gos_AddPoints(&p, 1, vec4(1,1,1,1), 4);
            for(int j=0; j<36;++j) {
                vec3 dp = p + 0.1*vec3(sin((float)j*10*M_PI/180.0f), cos((float)j*10*M_PI/180.0f), 0.0f); 
                gos_AddPoints(&dp, 1, vec4(1,1,1,1), 4);
            }

        }
#endif

        TEXTUREPTR texinfo;
#if 0
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
#endif 
     
        vec3* vpos = new vec3[vsx*vsy];
        int offset = 0;
        for(uint32_t y=0;y<vsy;++y) {
            for(uint32_t x=0;x<vsx;++x) {
                vec2 p = grid->vtx2pos(x, y);
                if(grid_verts[x + y*vsx].value_ == 9999.0f)
    				vpos[offset++] = vec3(p.x, p.y, 0);
			}
        }
        gos_AddPoints(vpos, offset, vec4(1,0,0,1), 4);
        int start = offset;
        for(uint32_t y=0;y<vsy;++y) {
            for(uint32_t x=0;x<vsx;++x) {
                vec2 p = grid->vtx2pos(x, y);
                if(grid_verts[x + y*vsx].value_ <= 0.0f)
    				vpos[offset++] = vec3(p.x, p.y, 0);
			}
        }
        gos_AddPoints(vpos+start, offset - start, vec4(0,0,1,1), 4);
        int start2 = offset;
        for(uint32_t y=0;y<vsy;++y) {
            for(uint32_t x=0;x<vsx;++x) {
                vec2 p = grid->vtx2pos(x, y);
                if(grid_verts[x + y*vsx].value_ > 0.0f && grid_verts[x + y*vsx].value_ != 9999.0f)
    				vpos[offset++] = vec3(p.x, p.y, 0);
			}
        }
        gos_AddPoints(vpos+start2, offset - start2, vec4(1,1,0,1), 4);
        delete[] vpos;
    
        gos_LockTexture(sdf_tex, 0, false, &texinfo);
        for(uint32_t y=0; y<vsy;++y) {
            float* row = (float*)texinfo.pTexture + y*texinfo.Pitch;
            SPHGridVertex* src_row = grid_verts + y*vsy;
            for(uint32_t x=0;x<vsx;++x) {
                row[x] = src_row[x].value_;
            }
        }
        gos_UnLockTexture(sdf_tex);

        boundary->UpdateTexturesByData();

	});

    HGOSRENDERMATERIAL sdf_material = sdf_mat_;
	ScheduleDebugDrawCommand(rfc, [surface_grid_tex, boundary, sdf_tex, sdf_material]() {
		//render_quad(surface_grid_tex, vec4(0.25f, 0.25f, -0.5f, 0.5f), nullptr);
		render_quad(sdf_tex, vec4(0.5f, 0.5f, -0.5f, 0.5f), sdf_material);

		//render_quad(g_sim->boundary_model_->getVolumeTexture(), vec4(0.25f, 0.25f, -0.5f, 0.5f), sdf_material);
		//render_quad(g_sim->boundary_model_->getNormalTexture(), vec4(0.25f, 0.25f, -0.5f, 0.5f), sdf_material);
		render_quad(boundary->getDistanceTexture(), vec4(0.25f, 0.25f, 0.25f, 0.5f), sdf_material);
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
    rp->mesh_.num_instances = num_particles;

	rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
    rp->mesh_ = *boundary_->getBoundaryMesh();
    rp->m_ = transform_->GetTransform();
	rp->is_debug_pass = 1;
	rp->debug_color = vec4(0,1,0,1);

    // update & draw surface mesh
    surface_->UpdateMesh(rfc);
   
	rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
    rp->m_ = transform_->GetTransform();
	rp->mesh_ = *surface_->getRenderMesh();

}

void initialize_particle_positions(SPHFluidModel* fm) {
    SPHParticle2D* particles = fm->particles_.data();
    const int count = (int)fm->particles_.size();
    const float radius = fm->radius_;
    vec2 offset = vec2(.2f, 4.0f*radius);
    int row_size = (int)(sqrtf((float)count) + 1.5f);////(int)(o->GetBounds().x / (2.0f*radius));
    int column_size = (count + row_size - 1) / row_size; 
    for(int y= 0; y<column_size; ++y) {
        for(int x= 0; x<row_size; ++x) {
            int idx = y*row_size + x;
            if(idx >= count)
                break;
            SPHParticle2D& p = particles[idx];
            float jitter = 0;// random(-0.01f, 0.01f);
            p.pos = offset + vec2(x*2.0f*radius + jitter, y*2.0f*radius);
            p.pressure = 0;
            p.density = 0;
            p.force = vec2(0,0);
            if(idx==0)
                p.vel = vec2(0.0f, 0.0f);
            else
                p.vel = vec2(0.0f, 0.0f);

        }
    }
}
