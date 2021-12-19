#include "renderer.h"
#include "sph_object.h"
#include "sph.h"
#include "sph_boundary.h"
#include "sph_emitter.h"
#include "sph_polygonize.h"
#include "sph_solver_df.h"
#include "sph_solver_pbd.h"
#include "res_man.h"
#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/camera_utils.h" // screen2world_vec

#include <cstddef> // offsetof

SPHBoundaryComponent* SPHBoundaryComponent::Create(const SPHSimulation *sim, const vec2 &dim, bool b_invert, bool b_dynamic) {

    SPHBoundaryComponent* c = new SPHBoundaryComponent();
    
	const float volume_map_cell_size = 0.025f;
    const bool b_is2d = true;
    c->boundary_ = new SPHBoundaryModel();
    ivec3 resolution = ivec3((int)(dim.x / volume_map_cell_size), (int)(dim.y / volume_map_cell_size), 1);
    c->boundary_->Initialize(vec3(dim.x, dim.y, 1), sim->radius(), sim->support_radius(), resolution, b_is2d, b_invert, b_dynamic);

    c->on_transformed_fptr_ = on_transformed;

    return c;
}

void SPHBoundaryComponent::Destroy(SPHBoundaryComponent* comp)
{
    delete comp->boundary_;
    delete comp;
}

void SPHBoundaryComponent::on_transformed(TransformComponent* comp) {
    SPHBoundaryComponent* bc = (SPHBoundaryComponent*)comp;  
    assert(bc->GetType() == ComponentType::kSPHBoundary);

	const quaternion q = bc->GetRotation();
	const vec3 p = bc->GetPosition();
	const vec3 s = bc->GetScale();
	bc->getBoundary()->setTransform(p, q, s);
}

void SPHBoundaryComponent::UpdateComponent(float dt) {
    //TransformComponent::UpdateComponent(dt);
	//const quaternion q = GetRotation();
	//const vec3 p = GetPosition();
	//const vec3 s = GetScale();
	//boundary_->setTransform(p, q, s);
}


SPHBoundaryObject::SPHBoundaryObject(const SPHSimulation* sim, const vec2 &dim, bool b_invert, bool b_dynamic)
{
    Tuple_.tr_ = AddComponent<TransformComponent>();

    Tuple_.bm_ = SPHBoundaryComponent::Create(sim, dim, b_invert, b_dynamic);
    AddComponent(Tuple_.bm_);
    addToSimulation(sph_get_simulation());

    Tuple_.bm_->SetParent(Tuple_.tr_);
}

void SPHBoundaryObject::addToSimulation(SPHSimulation* sim) {

    sim->addBoundary(Tuple_.bm_->getBoundary());
}

void SPHBoundaryObject::removeFromSimulation(SPHSimulation* sim) {
    sim->removeBoundary(Tuple_.bm_->getBoundary());
}

SPHBoundaryObject::~SPHBoundaryObject()
{
    delete RemoveComponent(Tuple_.tr_);
    auto c = RemoveComponent(Tuple_.bm_);
    SPHBoundaryComponent::Destroy(c);
}
void SPHBoundaryObject::Update(float dt) {

/*
    vec3 pos = Tuple_.tr_->GetPosition();
    quaternion rot = Tuple_.tr_->GetRotation();

    Tuple_.bm_->getBoundary()->setTransform(pos, quat_to_mat3(rot));
*/
}

void SPHBoundaryObject::InitRenderResources() {
    Tuple_.bm_->getBoundary()->InitializeRenderResources();
}

void SPHBoundaryObject::DeinitRenderResources() {

    // destroy render resource here
}

void SPHBoundaryObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	SPHBoundaryModel *bm = Tuple_.bm_->getBoundary();
	ScheduleRenderCommand(rfc, [bm]() { bm->UpdateTexturesByData(); });

	class RenderList *rl = rfc->rl_;
	RenderPacket *rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->mesh_ = *bm->getBoundaryMesh();

	//rp->m_ = Tuple_.tr_->GetTransform();
	rp->m_ = bm->getTransformMatrix();

	//rp->is_debug_pass = 1;
	rp->is_opaque_pass = 1;
	rp->debug_color = vec4(0, 1, 0, 1);
}

void initialize_particle_positions(SPHFluidModel* fm);

SPHSceneObject* SPHSceneObject::Create(const vec2& view_dim, int num_particles, const vec3& pos) {

    const bool b_is2d = true;

    SPHSceneObject* o = new SPHSceneObject();
    o->view_dim_ = view_dim;
    o->radius_ = 0.1f;

	// recommended by 2014_EG_SPH_STAR.pdf 7.1
	float grid_cell = o->radius_;
	vec2 grid_res = view_dim / grid_cell;
	ivec2 igrid_res = ivec2(min(grid_res.x, 128.0f), min(grid_res.y, 128.0f));
	o->grid_ = SPHGrid::makeGrid(igrid_res.x, igrid_res.y, view_dim);

    auto tr = o->AddComponent<TransformComponent>();
    tr->SetPosition(pos);
    o->transform_ = tr;
    o->b_initalized_rendering_resources = false;

    SPHFluidModel* fm = new SPHFluidModel();

    fm->density0_ = 1000;
    fm->radius_ = o->radius_;
    fm->support_radius_ = 3.0f * fm->radius_;
    fm->viscosity_ = 0;
    // slightly reduced square (cube for 3D)
    const float diameter = 2.0f * fm->radius_;
    if (b_is2d) {
        fm->volume_ = diameter * diameter;// * 0.8f;
    } else {
        fm->volume_ = diameter * diameter * diameter * 0.8f;
    }
	fm->particles_.resize(num_particles);
    fm->grid_ = o->grid_;
    initialize_particle_positions(fm);

    const vec2 boundary_size = vec2(10, 2);
    const float volume_map_cell_size = o->radius_;
    SPHBoundaryModel* bm = new SPHBoundaryModel();
    ivec3 resolution = ivec3((int)(boundary_size.x / volume_map_cell_size), (int)(boundary_size.y / volume_map_cell_size), 1);
    bm->Initialize(vec3(boundary_size, 10.0f), fm->radius_, fm->support_radius_, resolution, true, false, false);
    bm->setTransform(vec3(-0.25f*boundary_size, 0.0f), quaternion::identity(), vec3(1));

#if ENABLE_PARTICLE_EMITTER
    SPHEmitter* e = SPHEmitterSystem::createrEmitter();
    e->dir_ = normalize(vec2(1,1));
    e->pos_ = 0.5f * o->view_dim_;
    e->initial_vel_ = e->dir_ * 2.0f;
    e->rate_ = 1.0f;
    e->fluid_model_ = fm;
    o->emitters_.push_back(e);
#endif

    o->fluid_ = fm;
    o->boundaries_.push_back(bm);

    o->surface_ = new SPHSurfaceMesh();
    
    SPHSimulation* sim = sph_get_simulation();
    sim->addBoundary(bm);
    sim->setFluid(o->fluid_);

	return o;
}

SPHSceneObject::~SPHSceneObject() {
    for(auto e: emitters_) {
        SPHEmitterSystem::destroyEmitter(e);
    }
    delete surface_;
    delete[] part_indices_;
    delete[] part_flags_;
    delete grid_;
}

HGOSVERTEXDECLARATION get_sph_vdecl() {

	static gosVERTEX_FORMAT_RECORD sph_vdecl[] = {
		// SVD
		{0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{1, 2, false, sizeof(SVD), offsetof(SVD, uv), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{2, 3, false, sizeof(SVD), offsetof(SVD, normal), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},

		// instance data: stream 1
		{3, 2, false, sizeof(SPHInstVDecl), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{4, 2, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,vel), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{5, 2, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,force), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{6, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,density), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{7, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,pressure), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{8, 1, false, sizeof(SPHInstVDecl), offsetof(SPHInstVDecl,flags), gosVERTEX_ATTRIB_TYPE::kUNSIGNED_INT, 1},
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

    for(auto boundary: boundaries_) {
        boundary->InitializeRenderResources();
    }
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

    if(emitters_.size() && gos_GetKeyStatus(KEY_Q) == KEY_PRESSED) {
        emitters_[0]->enabled_ = true;
    }

    surface_->updateFromGrid(grid_);

    grid_->cur_vert_array_rt_ = grid_->cur_vert_array_;
	grid_->cur_vert_array_ = (grid_->cur_vert_array_ + 1) % (grid_->num_vert_arrays_);
}

extern void render_quad(uint32_t tex_id, const vec4& scale_offset, HGOSRENDERMATERIAL pmat);
void update_closest_dist_debug_line(struct RenderFrameContext *rfc);

void SPHSceneObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

	if (!b_initalized_rendering_resources) return;

    update_closest_dist_debug_line(rfc);

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
    auto boundaries = boundaries_;

    SPH_PBDDebugDrawSimData(fluid_, rfc->rl_);
    //SPH_DFDebugDrawSimData(fm);

	ScheduleRenderCommand(rfc, [num_particles, inst_vb, particles, grid_verts, grid, boundaries, surface_grid_tex, sdf_tex, vsx, vsy]() {
		const size_t bufsize = num_particles * sizeof(SPHParticle2D);
		// TODO: think about typed buffer wrapper
        int inst_buf_num_part = (int)(gos_GetBufferSizeBytes(inst_vb)/sizeof(SPHParticle2D));
        if(inst_buf_num_part < num_particles) {
            gos_ResizeBuffer(inst_vb, (uint32_t)(num_particles*1.5f));
        }

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

        (void)surface_grid_tex;
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
    
        gos_LockTexture(sdf_tex, 0, gosLockFlags_Write, &texinfo);
        for(uint32_t y=0; y<vsy;++y) {
            float* row = (float*)texinfo.pTexture + y*texinfo.Pitch;
            SPHGridVertex* src_row = grid_verts + y*vsy;
            for(uint32_t x=0;x<vsx;++x) {
                row[x] = src_row[x].value_;
            }
        }
        gos_UnLockTexture(sdf_tex);

        for(auto b: boundaries)
            b->UpdateTexturesByData();

	});
#if 0
    HGOSRENDERMATERIAL sdf_material = sdf_mat_;
	ScheduleDebugDrawCommand(rfc, [surface_grid_tex, boundaries, sdf_tex, sdf_material]() {
		//render_quad(surface_grid_tex, vec4(0.25f, 0.25f, -0.5f, 0.5f), nullptr);
		render_quad(sdf_tex, vec4(0.5f, 0.5f, -0.5f, 0.5f), sdf_material);

		//render_quad(g_sim->boundary_model_->getVolumeTexture(), vec4(0.25f, 0.25f, -0.5f, 0.5f), sdf_material);
		//render_quad(g_sim->boundary_model_->getNormalTexture(), vec4(0.25f, 0.25f, -0.5f, 0.5f), sdf_material);
        render_quad(boundaries[0]->getDistanceTexture(), vec4(0.25f, 0.25f, 0.25f, 0.5f), sdf_material);
	});
#endif

	class RenderList* rl = rfc->rl_;
	RenderPacket *rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
	// should we draw spheres into selection pass? does not work right now
	// need vertex shader variant which supports reading instance attribute(s)
	// see deferred_sph or better add define to deferred_sph which will be enabled in obj
	// is pass and will write to obj_id render target
	rp->is_selection_pass = 0;
    rp->m_ = transform_->GetTransform(); //mat4::scale(vec3(radius_));
	rp->mesh_ = *sphere_mesh_;
	rp->mesh_.mat_ = mat_;
	rp->mesh_.inst_vb_ = inst_vb_[cur_inst_vb_];
    rp->mesh_.vdecl_ = vdecl_;
    rp->mesh_.num_instances = num_particles;

	for (auto b : boundaries) {
		rp = rl->AddPacket();
		memset(rp, 0, sizeof(RenderPacket));
		rp->mesh_ = *b->getBoundaryMesh();
		rp->m_ = b->getTransformMatrix();
		rp->is_debug_pass = 1;
		rp->debug_color = vec4(0, 1, 0, 1);
	}

	// update & draw surface mesh
    surface_->UpdateMesh(rfc);
   
	rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
    rp->m_ = transform_->GetTransform();
	rp->mesh_ = *surface_->getRenderMesh();

}

void update_closest_dist_debug_line(struct RenderFrameContext *rfc) {

#define ENABLE_CLOSEST_DISTANCE_DEBUG 1
#if ENABLE_CLOSEST_DISTANCE_DEBUG 
    const SPHSimulation* sim = sph_get_simulation();
    if(sim) {
        int XDelta, YDelta, WheelDelta;
        float XPos, YPos;
        DWORD buttonsPressed;
        gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);

	    const vec2 mouse_screen_pos = 2 * vec2(XPos, 1 - YPos) - vec2(1);
        const vec3 dir = screen2world_vec(rfc->inv_view_, rfc->inv_proj_, mouse_screen_pos);

        const vec3 ws_cam_pos = (rfc->inv_view_ * vec4(0, 0, 0, 1)).xyz();
        const vec3 pos = ray_plane_intersect(dir, ws_cam_pos, make_plane(vec3(0,0,1),vec3(0,0,0)));

        SPHBoundaryModel *bm;
        float dist;
        vec2 normal;

        dist = FLT_MAX;
        normal = vec2(0,0);
        for(SPHBoundaryModel* model : sim->boundary_models_) {
            float cur_dist = model->getDistance2D(pos.xy());
            if(cur_dist < dist) {
                dist = cur_dist;
                normal = model->getNormal2D(pos.xy());
                bm = model;
            }
        }

        assert(bm);

		vec2 closest_pos = pos.xy() - normal * dist;

        vec3 lb = vec3(pos.xy(), -0.05f);
        vec3 le = vec3(closest_pos, -0.05f);
        rfc->rl_->addDebugLine(lb, le, vec4(0.5f,1,1,1));
    }
#endif
}

void initialize_particle_positions(SPHFluidModel* fm) {
    SPHParticle2D* particles = fm->particles_.data();
    const int count = (int)fm->particles_.size();
    const float radius = fm->radius_;
    vec2 offset = vec2(.2f, 4.0f*radius + 4.0f);
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
