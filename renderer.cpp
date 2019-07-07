#include "renderer.h"

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

