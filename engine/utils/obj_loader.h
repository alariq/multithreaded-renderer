#include "hashing.h"
#include "utils/vec.h"

#include <cassert>
#include <cstring>
#include <vector>
#include <string>
#include <unordered_map>

struct ObjVertexId {
    int32_t p;
    int32_t t;
    int32_t n;

    ObjVertexId(int32_t p_, int32_t t_, int32_t n_):
        p(p_), t(t_), n(n_) { }
};

struct ObjVertex {
    vec3 p;
    vec2 t;
    vec3 n;
};

struct ObjFile {
    std::vector<vec3> p;
    std::vector<vec2> t;
    std::vector<vec3> n;
    std::vector<ObjVertexId> faces;
    std::string material_name;
};

struct obj_vertex_id_comparator {
    bool operator()(ObjVertexId const& a, ObjVertexId const& b) const noexcept {
        return 0 == memcmp(&a, &b, sizeof(ObjVertexId));
    }
};

struct obj_vertex_id_hash
{
    std::size_t operator()(ObjVertexId const& o) const noexcept
    {
        return stl_container_hash((const uint8_t*)&o, sizeof(ObjVertexId));
    }
};


ObjFile* load_obj_from_file(const char* file);
void create_index_and_vertex_buffers(const ObjFile* obj, uint32_t** ib, uint32_t * ib_count, ObjVertex** vb, uint32_t* vb_count);

template <typename MeshBuffer>
inline void create_index_and_vertex_buffers(const ObjFile* obj, MeshBuffer& mb) {

    std::unordered_map<ObjVertexId, int32_t, obj_vertex_id_hash,
                       obj_vertex_id_comparator>
        vertex_map(50);

    uint32_t num_vertices = 0;
    const uint32_t num_indices = (uint32_t)obj->faces.size();
    mb.allocate_ib(num_indices);

    for(size_t i=0; i < num_indices; ++i)
    {
        auto it = vertex_map.find(obj->faces[i]);
        if(it == vertex_map.end())
        {
            mb.i(i, num_vertices);
            vertex_map.insert(std::make_pair(obj->faces[i], num_vertices++));
        } else {
            mb.i(i, it->second);
        }
    }

    mb.allocate_vb(num_vertices);
    for(auto v_id: vertex_map) {
        // indices start from 1
        assert(v_id.first.p);
        vec3 p = obj->p[v_id.first.p - 1];
        vec2 t = v_id.first.t != 0 ? obj->t[v_id.first.t - 1] : vec2(0.0f);
        vec3 n = v_id.first.n != 0 ? obj->n[v_id.first.n - 1] : vec3(0.0f);

        mb.p(v_id.second, p);
        mb.uv(v_id.second, t);
        mb.n(v_id.second, n);
    }
}
