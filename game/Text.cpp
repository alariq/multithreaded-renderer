#include "../obj_model.h"
#include "engine/utils/timing.h"
#include "Text.h"
#include "gameos.hpp"

void GameTextComp::InitRenderResources() {

    //font_tex_handle = gos_NewTextureFromFile(gos_TextureFormat::gos_Texture_RGBA8, "./data/roboto_medium_24.fnt.bmp");
    //assert(font_tex_handle != 
    font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_64");
    small_font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_24");
	state_ = Component::kInitialized;
}

void GameTextComp::DeinitRenderResources() {
    gos_DeleteFont(font_handle_);
    state_ = Component::kUninitialized;
}

void GameTextComp::AddRenderPackets(struct RenderFrameContext * rfc) const {

    uint32_t i = (int)(intensity_ * 255.0f + 0.5f);
    //                      b        g         r         a
    uint32_t color = i | (i<<8) | (i<<16) | (i<<24) | 0xFF000000;

    char pposbuf[32];
    vec4 ppos = rfc->proj_ * rfc->view_ * vec4(GetPosition(), 1);
    // TODO: using viewport here might not work if we render to some pass which uses different WxH, need to use per pass current viewport
    vec2 screenpos = (vec2(ppos.x/ppos.w, ppos.y/ppos.w)*vec2(0.5f, -0.5f) + vec2(0.5f))*rfc->viewport;
    sprintf(pposbuf, "%.2f, %.2f, %.2f %.2f\n", screenpos.x, screenpos.y, ppos.z, ppos.w);

    char buf[256];
    sprintf(buf, "%.2f Insert coin %s", frame_time_ms_, pposbuf);



    rfc->rl_->addTextPacket(buf, font_handle_, color, 64, 10, 0);

    rfc->rl_->addTextPacket("Hello!", small_font_handle_, color, 24, screenpos.x, screenpos.y);
}

void GameTextComp::Initialize() {
    intensity_ = 0.5f;
    frame_time_ms_ = 0.0f;
}

void GameTextComp::Deinitialize() { 
}


void GameTextComp::UpdateComponent(float dt) {
    // TODO: add game time (does not grow when game is paused)
    double time_s = 0.001*timing::ticks2ms(timing::gettickcount());
    frame_time_ms_ = dt*1000.0f;
    intensity_ = saturate(0.25f*sin(time_s*0.5f*M_PI/180.0f) + 0.75f);

    //vec3 pos = GetPosition();
    //sprintf(text, "%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.z);
}
