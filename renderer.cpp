#include "renderer.h"
#include <cassert>

TSQueue<RenderList*> gFreeRenderLists;

RenderList* AcquireRenderList()
{
    RenderList* rl = gFreeRenderLists.pop();
    if(!rl)
    {
        // all render lists are taken get new one
        rl = new RenderList();
    }

    rl->Acquire();
    return rl;
}

void ReleaseRenderList(RenderList* rl) {
    assert(rl && rl->IsAcquired());

    rl->Release();
    rl->GetRenderPackets().resize(0);
    gFreeRenderLists.push(rl);
}

void DeleteRenderLists() {

    while(gFreeRenderLists.size()) {
        RenderList* rl = gFreeRenderLists.pop();
        delete rl;
    }
}

void ScheduleRenderCommand(RenderFrameContext *rfc, std::function<void(void)>&& cmd)
{
    assert(rfc);
    rfc->commands_.push_back(cmd);
}

HGOSVERTEXDECLARATION get_svd_vdecl() {
    static const gosVERTEX_FORMAT_RECORD simple_vdecl[] = {
        {0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
        {1, 2, false, sizeof(SVD), offsetof(SVD, uv),
         gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
        {2, 3, false, sizeof(SVD), offsetof(SVD, normal),
         gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
    };

    static auto vdecl = gos_CreateVertexDeclaration(
        simple_vdecl, sizeof(simple_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}

HGOSVERTEXDECLARATION get_pos_only_vdecl() {

    static const gosVERTEX_FORMAT_RECORD pos_only_vdecl[] = {
        {0, 3, false, sizeof(vec3), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
    };

    static auto vdecl = gos_CreateVertexDeclaration(
        pos_only_vdecl,
        sizeof(pos_only_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
};

HGOSVERTEXDECLARATION get_quad_vdecl() {
    static const gosVERTEX_FORMAT_RECORD quad_vdecl[] = {
        {0, 2, false, sizeof(QVD), 0, gosVERTEX_ATTRIB_TYPE::FLOAT, 0},
        //{1, 2, false, sizeof(QVD), offsetof(QVD, uv),
        //gosVERTEX_ATTRIB_TYPE::FLOAT
        //}
    };

    static auto vdecl = gos_CreateVertexDeclaration(
        quad_vdecl, sizeof(quad_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}
