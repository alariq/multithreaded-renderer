#include "res_man.h"
#include "renderer.h"
#include "gameos.hpp" // DWORD
#include "utils/obj_loader.h"
#include "utils/intersection.h" // aabb

#include <string>
#include <unordered_map>

std::unordered_map<std::string, RenderMesh*> g_world_meshes;
std::unordered_map<std::string, DWORD> g_world_textures;
static bool is_res_man_initialized = false;

template <typename IB_t = uint16_t>
struct SVDAdapter {
    typedef IB_t ib_type;

    SVD *vb_ = nullptr;
    ib_type *ib_ = nullptr;
    size_t vb_size_ = 0;
    size_t ib_size_ = 0;
    size_t offset_ = 0;

    enum { kVertexSize = sizeof(SVD) };

    void allocate_vb(size_t size) {
        SVD* new_vb = new SVD[vb_size_ + size];
        memcpy(new_vb, vb_, sizeof(SVD)*vb_size_);
        delete[] vb_;
        vb_ = new_vb;
        vb_size_ += size;
    }
    void allocate_ib(size_t size) {
        ib_type* new_ib = new ib_type[ib_size_ + size];
        memcpy(new_ib, ib_, sizeof(ib_type)*ib_size_);
        delete[] ib_;
        ib_ = new_ib;
        ib_size_ += size;
    }
    ~SVDAdapter() {
        delete[] vb_;
        delete[] ib_;
    }

    AABB get_aabb() const {
        AABB aabb(vec3(0),vec3(0));
        if(vb_size_ > 0) {
            aabb.min_ = vb_[0].pos;
            aabb.max_ = vb_[0].pos;
            for(size_t i=1; i<vb_size_; ++i) {
                aabb.min_ = min(vb_[i].pos, aabb.min_);
                aabb.max_ = max(vb_[i].pos, aabb.max_);
            }
        }
        return aabb;
    }

    void set_offset(size_t offset) { offset_ = offset; }
    void p(unsigned int i, vec3 p) { vb_[i+offset_].pos = p; }
    void n(unsigned int i, vec3 n) { vb_[i+offset_].normal = n; }
    void uv(unsigned int i, vec2 uv) { vb_[i+offset_].uv = uv; }
    void i(unsigned int i, ib_type idx) { ib_[i] = idx; }
};

template <typename MeshBuffer>
static RenderMesh *render_mesh_from_mesh_buffer(const MeshBuffer &mb,
                                           HGOSVERTEXDECLARATION vdecl) {
    RenderMesh *mesh = new RenderMesh();
    mesh->vdecl_ = vdecl;
    if (mb.ib_) {
        mesh->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                     gosBUFFER_USAGE::STATIC_DRAW,
                                     sizeof(typename MeshBuffer::ib_type), (uint32_t)mb.ib_size_, mb.ib_);
    } else {
        mesh->ib_ = nullptr;
    }

    assert(mb.vb_);
    mesh->vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
                         mb.kVertexSize, (uint32_t)mb.vb_size_, mb.vb_);

    mesh->prim_type_ = PRIMITIVE_TRIANGLELIST;
    mesh->aabb_ = mb.get_aabb();
    return mesh;
}

static RenderMesh* CreateCubeRenderMesh() {
        SVDAdapter<> svd_adapter;
        generate_cube(svd_adapter, vec3(1), vec3(0));
        return render_mesh_from_mesh_buffer(svd_adapter, get_svd_vdecl());
}

static RenderMesh* CreateRenderMesh(const ObjFile* obj) {
        SVDAdapter<uint32_t> svd_adapter;
        create_index_and_vertex_buffers(obj, svd_adapter);
        return render_mesh_from_mesh_buffer(svd_adapter, get_svd_vdecl());
}

static RenderMesh* CreateFSQuadRenderMesh() {
    constexpr const size_t NUM_VERT = 4;
    constexpr const size_t NUM_IND = 6;

    QVD vb[NUM_VERT] = {{vec2(-1.0f, -1.0f)},
                        {vec2(-1.0f, 1.0f)},
                        {vec2(1.0f, -1.0f)},
                        {vec2(1.0f, 1.0f)}};
    uint16_t ib[NUM_IND] = {0, 2, 3, 0, 3, 1};

    RenderMesh* fs_quad = new RenderMesh();
    fs_quad->vdecl_ = get_quad_vdecl();
    fs_quad->ib_ =
        gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW,
                         sizeof(uint16_t), NUM_IND, ib);
    fs_quad->vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
                         sizeof(QVD), NUM_VERT, vb);

    fs_quad->prim_type_ = PRIMITIVE_TRIANGLELIST;
    return fs_quad;
}

static RenderMesh *CreateSphereRenderMesh() {
    SVDAdapter<> svd_adapter;
    generate_sphere(svd_adapter, 5);
    return render_mesh_from_mesh_buffer(svd_adapter, get_svd_vdecl());
}

static RenderMesh *CreateAxesRenderMesh() {

    SVDAdapter<> svd_adapter;
    generate_axes(svd_adapter);
    return render_mesh_from_mesh_buffer(svd_adapter, get_svd_vdecl());
}

static RenderMesh *CreateTorusRenderMesh() {

    SVDAdapter<> svd_adapter;
    generate_torus(svd_adapter, 1.0f, 0.035f, 32, 32);
    return render_mesh_from_mesh_buffer(svd_adapter, get_svd_vdecl());
}

void initialize_res_man() { 

    assert(!is_res_man_initialized);

    // create default texture
    DWORD def_tex = gos_NewTextureFromFile(gos_Texture_Detect,
                                           "data/textures/texture_density.tga");
    assert(def_tex);
    g_world_textures.insert(std::make_pair("default", def_tex));

    // create default mesh
    RenderMesh *def = CreateCubeRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("cube", def));
    g_world_meshes.insert(std::make_pair("default", def));

    def = CreateFSQuadRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("fs_quad", def));

    def = CreateSphereRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("sphere", def));

    def = CreateAxesRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("axes", def));

    def = CreateTorusRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("torus", def));

    gos_AddRenderMaterial("coloured_quad");
    gos_AddRenderMaterial("textured_quad");
    gos_AddRenderMaterial("debug");

    is_res_man_initialized = true;
}
void finalize_res_man() {

    assert(is_res_man_initialized);

    for(auto tex_id: g_world_textures)
        gos_DestroyTexture(tex_id.second);

    g_world_textures.clear();

    is_res_man_initialized = false;
}

DWORD res_man_load_texture(const std::string& name) {
    DWORD tex_id = 0;
    auto tex_it = g_world_textures.find(name);
    if(tex_it!=g_world_textures.end()) {
        tex_id = tex_it->second;
    } else {
        std::string tex_fname = "data/textures/" + name + ".tga";
        tex_id = gos_NewTextureFromFile(gos_Texture_Detect, tex_fname.c_str());
        if(0 == tex_id) {
            printf("Failed to load: %s\n", tex_fname.c_str());
            tex_id = g_world_textures["default"];
        } else
            g_world_textures.insert(std::make_pair(name, tex_id));
    }
    return tex_id;
}

RenderMesh* res_man_load_mesh(const std::string mesh_name) {

    auto it = g_world_meshes.find(mesh_name);
    if(it!=g_world_meshes.end())
        return it->second;

    RenderMesh* mesh = nullptr;

    std::string fname = "data/meshes/" + mesh_name + ".obj";
    ObjFile* obj = load_obj_from_file(fname.c_str());
    if(!obj) {
        printf("Failed to load: %s\n", fname.c_str());
        mesh = g_world_meshes["cube"];
        assert(mesh);
    } else {
        mesh = CreateRenderMesh(obj);
        mesh->tex_id_ = res_man_load_texture(mesh_name);
    }
    g_world_meshes.insert(std::make_pair(mesh_name, mesh));

    return mesh;
}

