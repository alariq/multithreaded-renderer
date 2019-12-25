#include <cassert>
#include <cstring>
#include "utils/vec.h"
#include <vector>
#include <string>

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


ObjFile* load_obj_from_file(const char* file);
void create_index_and_vertex_buffers(const ObjFile* obj, uint32_t** ib, uint32_t * ib_count, ObjVertex** vb, uint32_t* vb_count);
