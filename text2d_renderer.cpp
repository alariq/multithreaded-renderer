#include "text2d_renderer.h"
#include "gameos.hpp"
#include "engine/profiler/profiler.h"

void RenderText2D(const TextRenderPacketList_t& rpl, const mat4& /*view*/, const mat4&/* proj*/)
{
    SCOPED_ZONE_N(Text2DDraw, 0);

    gos_SetRenderState(gos_State_ZCompare, 0);
    gos_SetRenderState(gos_State_Culling, gos_Cull_None);
    gos_SetRenderState(gos_State_ZWrite, true);
    //gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);

    for(auto& tp: rpl)
    {
        gos_TextSetAttributes(tp.font_handle, tp.colour, tp.Size, tp.WordWrap, tp.Proportional, tp.Bold, tp.Italic, tp.WrapType, tp.DisableEmbeddedCodes);
        gos_TextSetPosition(tp.PosX, tp.PosY);
        gos_TextDraw((const char*)tp.text);
    }

    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_ZWrite, 1);
}

