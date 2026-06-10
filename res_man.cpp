#include "res_man.h"
#include "renderer.h"
#include "gameos.hpp" // DWORD
#include "utils/obj_loader.h"
#include "utils/intersection.h" // aabb
#include "utils/logging.h"
#include "utils/gl_utils.h"

#include <string>
#include <unordered_map>
#include <memory>

struct TextureRes {
    std::string name_;
    std::string path_;
    struct {
        u32 id;
    } rd;

    TextureRes(const std::string& name, const std::string& path):name_(name), path_(path) {
        rd.id = 0;
    }

    TextureRes(const TextureRes& o) = delete;
    TextureRes& operator= (const TextureRes& o) = delete;

    //TODO: make it non-copyable? but actually should be a ref counted object
    // and we only shold return its "handle"
};

//TODO: make it so we only need gpu id when we actually have access to TextureRes internals
u32 texture_res_get_gpu_id(TextureHandle h) {
    return h ? h->rd.id : 0;
}

std::unordered_map<std::string, std::pair<StaticMesh*, int>> g_world_meshes2;
std::unordered_map<std::string, TextureRes*> g_world_textures2;
static bool is_res_man_initialized = false;

std::vector<std::function<void(void)> > g_rt_requests;
std::vector<std::pair<std::atomic_int*, StaticMesh*>> g_rt_barriers;

template <typename IB_t = uint16_t>
struct SVDAdapter {
    typedef IB_t ib_type;

    SVD *vb_ = nullptr;
    ib_type *ib_ = nullptr;
    size_t vb_size_ = 0;
    size_t ib_size_ = 0;
    size_t offset_ = 0;

    enum { kVertexSize = sizeof(SVD) };

    SVDAdapter(){};

    SVDAdapter(SVDAdapter&& o) {
        vb_ = o.vb_;
        ib_ = o.ib_;
        vb_size_ = o.vb_size_;
        ib_size_ = o.ib_size_;
        offset_ = o.offset_;
        o.vb_ = nullptr;
        o.ib_ = nullptr;
    }

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
static StaticMesh *static_mesh_from_mesh_buffer(const MeshBuffer* mb) {
    StaticMesh *mesh = new StaticMesh();

    mesh->prim_type_ = PRIMITIVE_TRIANGLELIST;
    mesh->vb_count_ = -1;
    mesh->vb_first_ = 0;
    mesh->two_sided_ = 0;
    mesh->aabb_ = mb->get_aabb();

    g_rt_requests.push_back([mesh = mesh, mb](){

        mesh->rd.vdecl_ = get_svd_vdecl();

        if (mb->ib_) {
            mesh->rd.ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX,
                gosBUFFER_USAGE::STATIC_DRAW,
                sizeof(typename MeshBuffer::ib_type), (uint32_t)mb->ib_size_, mb->ib_);
        } else {
            mesh->rd.ib_ = nullptr;
        }

        gosASSERT(mb->vb_);
        mesh->rd.vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
                mb->kVertexSize, (uint32_t)mb->vb_size_, mb->vb_);

        mesh->rd.inst_vb_ = 0;

        delete mb;
    });

    return mesh;
}

static void destroy_static_mesh_rt(StaticMesh* mesh) {

    mesh->rd.vdecl_ = nullptr;
    if(mesh->rd.ib_)
        gos_DestroyBuffer(mesh->rd.ib_);
    mesh->rd.ib_ = nullptr;

    gosASSERT(mesh->rd.vb_);
    gos_DestroyBuffer(mesh->rd.vb_);

    gosASSERT(mesh->rd.inst_vb_ == 0);
}

static StaticMesh* CreateCubeStaticMesh() {
        auto svd_adapter = new SVDAdapter<uint32_t>();
        generate_cube(*svd_adapter, vec3(1), vec3(0));
        return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh* CreateStaticMesh(const ObjFile* obj) {
        auto svd_adapter = new SVDAdapter<uint32_t>();
        create_index_and_vertex_buffers(obj, *svd_adapter);
        return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh* CreateFSQuadStaticMesh() {

    StaticMesh* fs_quad = new StaticMesh();

    fs_quad->prim_type_ = PRIMITIVE_TRIANGLELIST;
    fs_quad->vb_count_ = -1;
    fs_quad->two_sided_ = 0;
    fs_quad->vb_first_ = 0;

    g_rt_requests.push_back([fs_quad](){

        constexpr const size_t NUM_VERT = 4;
        constexpr const size_t NUM_IND = 6;

        QVD vb[NUM_VERT] = {{vec2(-1.0f, -1.0f)},
        {vec2(-1.0f, 1.0f)},
        {vec2(1.0f, -1.0f)},
        {vec2(1.0f, 1.0f)}};
        uint16_t ib[NUM_IND] = {0, 2, 3, 0, 3, 1};

        fs_quad->rd.vdecl_ = get_quad_vdecl();
        fs_quad->rd.ib_ =
        gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW,
                sizeof(uint16_t), NUM_IND, ib);
        fs_quad->rd.vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW,
                sizeof(QVD), NUM_VERT, vb);
    });

    return fs_quad;
}

// facing positive Z axis
static StaticMesh* CreateXY_PosZ_QuadStaticMesh() {
    auto svd_adapter = new SVDAdapter<uint32_t>();
    generate_quad(*svd_adapter, vec3(-1,1,1), vec3(0.5f, -0.5f, 0.0f), 0);
    return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh* CreateXYQuadStaticMesh() {
    auto svd_adapter = new SVDAdapter<uint32_t>();
    generate_quad(*svd_adapter, vec3(1,1,1), -vec3(0.5f, 0.5f, 0.0f), 0);
    return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh* CreateSphereStaticMesh() {
    auto svd_adapter = new SVDAdapter<uint32_t>();
    generate_sphere(*svd_adapter, 5);
    return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh *CreateAxesStaticMesh() {
    auto svd_adapter = new SVDAdapter<uint32_t>();
    generate_axes(*svd_adapter);
    return static_mesh_from_mesh_buffer(svd_adapter);
}

static StaticMesh *CreateTorusStaticMesh() {
    auto svd_adapter = new SVDAdapter<uint32_t>();
    generate_torus(*svd_adapter, .5f, 0.035f, 32, 32);
    return static_mesh_from_mesh_buffer(svd_adapter);
}

static TextureHandle CreateTexture(const char* name, const char* path) {
    TextureRes* tex_res = new TextureRes(name, path);

    g_rt_requests.push_back([tex_res = tex_res](){
        tex_res->rd.id = gos_NewTextureFromFile(tex_res->path_.c_str(), gosHint_Gamma);
        assert(tex_res->rd.id);
    });

    return tex_res;
}

void initialize_res_man() { 

    assert(!is_res_man_initialized);

    // create default texture
    const char* def_path = "data/textures/notfound.tga";
    DWORD def_tex = gos_NewTextureFromFile(def_path, gosHint_Gamma);
    gosASSERT(def_tex);
    TextureHandle def_tex2 = CreateTexture("default", def_path);

    g_world_textures2.insert(std::make_pair("default", def_tex2));

    // create default mesh
    StaticMesh *def2 = CreateCubeStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("default", std::make_pair(def2, 1)));

    // eah.. no ref counts, so cannot just reuse pointer here as we delete them twice then
    StaticMesh *cube2 = CreateCubeStaticMesh();
    cube2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("cube", std::make_pair(cube2, 1)));

    def2 = CreateFSQuadStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("fs_quad", std::make_pair(def2, 1)));

    def2 = CreateXY_PosZ_QuadStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("xy_quad", std::make_pair(def2, 1)));

    def2 = CreateSphereStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("sphere", std::make_pair(def2, 1)));

    def2 = CreateAxesStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("axes", std::make_pair(def2, 1)));

    def2 = CreateTorusStaticMesh();
    def2->tex_handle_ = def_tex2;
    g_world_meshes2.insert(std::make_pair("torus", std::make_pair(def2, 1)));

    g_rt_requests.push_back([](){
        gos_AddRenderMaterial("coloured_quad");
        gos_AddRenderMaterial("textured_quad");
        gos_AddRenderMaterial("debug");
    });

    // actually shoul wait for above materials  to load?
    is_res_man_initialized = true;
}
void finalize_res_man() {

    assert(is_res_man_initialized);

    for(auto p: g_world_textures2) {
        gos_DestroyTexture(p.second->rd.id);
        delete p.second;
    }
    g_world_textures2.clear();

    while(g_world_meshes2.size()) {
        auto& p = *g_world_meshes2.begin();
        StaticMesh* m = p.second.first; 
        int refcnt = p.second.second;
        if(refcnt != 1) {
            log_error("Error: expected mesh %s ref count 1, got: %d, force destroy\n", p.first.c_str(), refcnt);
            p.second.second = 1;
        }
        res_man_release_mesh2(m);
    }
    g_world_meshes2.clear();

    //HACK:
    RenderFrameContext nullctx;
    res_man_schedule_rt_requests(&nullctx);
    for (auto& cmd : nullctx.commands_) {
        cmd();
    }
    res_man_update();

    printf("rt requests: %d rt barriers:%d\n", 
        (int)g_rt_requests.size(), (int)g_rt_barriers.size());

    is_res_man_initialized = false;
}

TextureHandle res_man_load_texture2(const std::string& name) {
    TextureRes* tex_res = nullptr;
    auto tex_it = g_world_textures2.find(name);
    if(tex_it!=g_world_textures2.end()) {
        tex_res = tex_it->second;
    } else {
        tex_res = new TextureRes(name, std::string("data/textures/") + name + ".tga");

        g_rt_requests.push_back([tex_res](){
            tex_res->rd.id = gos_NewTextureFromFile(tex_res->path_.c_str(), gosHint_Gamma);
            if(tex_res->rd.id) {
                g_world_textures2.insert(std::make_pair(tex_res->name_, tex_res));
            } else {
                log_error("Failed to load: %s\n", tex_res->path_.c_str());
                tex_res->rd.id = 0;
            }
        });
    }
    return tex_res;
}

StaticMesh* res_man_load_mesh2(const std::string& mesh_name) {

    auto it = g_world_meshes2.find(mesh_name);
    if(it!=g_world_meshes2.end()) {
        it->second.second++;
        return it->second.first;
    }

    if(mesh_name == "sphere") {
        puts("");
    }

    StaticMesh* mesh = nullptr;

    std::string fname = "data/meshes/" + mesh_name + ".obj";
    ObjFile* obj = load_obj_from_file(fname.c_str());
    if(!obj) {
        printf("Failed to load: %s\n", fname.c_str());
        mesh = g_world_meshes2["cube"].first;
        g_world_meshes2["cube"].second++;
        assert(mesh);
    } else {
        mesh = CreateStaticMesh(obj);
        mesh->tex_handle_ = res_man_load_texture2(mesh_name);
        delete obj;
        g_world_meshes2.insert(std::make_pair(mesh_name, std::make_pair(mesh, 1)));
    }

    return mesh;
}

// TODO: should be thread safe, but for now only called from main thread
void res_man_release_mesh2(struct StaticMesh* mesh) {
    for(auto& wm: g_world_meshes2) {
        if(wm.second.first == mesh) {
            assert(wm.second.second >= 1);
            wm.second.second--;
            if(wm.second.second == 0) {

                std::atomic_int* barrier_ptr = new std::atomic_int(0);// :( make pretty later
                g_rt_barriers.push_back(std::make_pair(barrier_ptr, mesh)); 
                g_rt_requests.push_back([mesh, barrier_ptr]() {
                    destroy_static_mesh_rt(mesh);
                    barrier_ptr->store(1, std::memory_order_release); // this should be atomic, with release semantic...
                });
                // erase inside for() will cause undefined behaviour,
                // but we break after first erase, so it is ok.
                g_world_meshes2.erase(wm.first);
                break;
            }
        }
    }
}

void res_man_update() {

    auto b = std::begin(g_rt_barriers);
    auto e = std::end(g_rt_barriers);

    for(auto it=b;it!=e;++it) {
        if(it->first->load(std::memory_order_acquire)) { // and here acquire
            printf("destroying resource\n");
            delete it->second;
            delete it->first;
            it->first = nullptr; // mark for removal below
        }
    }
    g_rt_barriers.erase(std::remove_if(b, e, [](auto& p) { return p.first == nullptr; }), e);
}

void res_man_schedule_rt_requests(struct RenderFrameContext* rfc) {
    for(auto& req: g_rt_requests) {
        ScheduleRenderCommand(rfc, std::move(req));
    }
    g_rt_requests.clear();
}
