#include "res_man.h"
#include "renderer.h"
#include "gameos.hpp" // DWORD
#include "utils/obj_loader.h"

#include <string>
#include <unordered_map>

std::unordered_map<std::string, RenderMesh*> g_world_meshes;
std::unordered_map<std::string, DWORD> g_world_textures;
static bool is_res_man_initialized = false;

static RenderMesh* CreateCubeRenderMesh(const char* /*res*/) {

        constexpr const size_t NUM_VERT = 36; 
        constexpr const size_t NUM_IND = 36; 

        SVD vb[NUM_VERT];
        gen_cube_vb(&vb[0], NUM_VERT);

        RenderMesh* ro = new RenderMesh();

        uint16_t ib[NUM_IND];
        for(uint32_t i=0;i<NUM_IND;++i)
            ib[i] = i;

        ro->vdecl_ = get_svd_vdecl();
        ro->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                   gosBUFFER_USAGE::STATIC_DRAW,
                                   sizeof(uint16_t), NUM_IND, ib);
        ro->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                   gosBUFFER_USAGE::STATIC_DRAW, sizeof(SVD),
                                   NUM_VERT, vb);

        return ro;
}

static RenderMesh* CreateRenderMesh(const ObjFile* obj) {

        RenderMesh* ro = new RenderMesh();
        uint32_t* ib;
        ObjVertex* vb;
        uint32_t ib_count, vb_count;
        create_index_and_vertex_buffers(obj, &ib, &ib_count, &vb, &vb_count);

        static_assert(sizeof(SVD) == sizeof(ObjVertex), "Vetex structure sizez are different");
        ro->vdecl_ = get_svd_vdecl();

        ro->ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                                   gosBUFFER_USAGE::STATIC_DRAW,
                                   sizeof(uint32_t), ib_count, ib);
        ro->vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX,
                                   gosBUFFER_USAGE::STATIC_DRAW, sizeof(ObjVertex),
                                   vb_count, vb);
        delete[] ib;
        delete vb;

        return ro;
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

    return fs_quad;
}

static RenderMesh *CreateSphereRenderMesh() {
    struct SVDAdapter {
        SVD *vb_ = nullptr;
        uint16_t *ib_ = nullptr;
        size_t vb_size_ = 0;
        size_t ib_size_ = 0;

        void allocate_vb(size_t size) {
            delete vb_;
            vb_ = new SVD[size];
            vb_size_ = size;
        }
        void allocate_ib(size_t size) {
            delete ib_;
            ib_ = new uint16_t[size];
            ib_size_ = size;
        }
        void p(unsigned int i, vec3 p) { vb_[i].pos = p; }
        void n(unsigned int i, vec3 n) { vb_[i].normal = n; }
        void uv(unsigned int i, vec2 uv) { vb_[i].uv = uv; }
        void i(unsigned int i, uint16_t idx) { ib_[i] = idx; }
    };

    SVDAdapter svd_adapter;
    generate_sphere(svd_adapter, 5);
    RenderMesh* sphere = new RenderMesh();
    sphere->vdecl_ = get_svd_vdecl();
    sphere->ib_ = gos_CreateBuffer(
        gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(uint16_t),
        svd_adapter.ib_size_, svd_adapter.ib_);
    sphere->vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
                         sizeof(SVD), svd_adapter.vb_size_, svd_adapter.vb_);

    return sphere;
}

void initialize_res_man() { 

    assert(!is_res_man_initialized);

    // create default texture
    DWORD def_tex = gos_NewTextureFromFile(gos_Texture_Detect,
                                           "data/textures/texture_density.tga");
    assert(def_tex);
    g_world_textures.insert(std::make_pair("default", def_tex));

    // create default mesh
    RenderMesh *def = CreateCubeRenderMesh("cube");
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("cube", def));
    g_world_meshes.insert(std::make_pair("default", def));

    def = CreateFSQuadRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("fs_quad", def));

    def = CreateSphereRenderMesh();
    def->tex_id_ = def_tex;
    g_world_meshes.insert(std::make_pair("sphere", def));

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

