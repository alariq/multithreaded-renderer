#include "../obj_model.h"
#include "engine/utils/timing.h"
#include "Text.h"
#include "Time.h"
#include "gameos.hpp"

void GameTextComp::InitRenderResources() {

    //font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_64");
    font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_24");
    assert(font_handle_);
    small_font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_24");
    assert(small_font_handle_);
    //slug_font_handle_ = gos_LoadSlugFont("./data/fonts/roboto_medium.slug");
    //slug_font_handle_ = gos_LoadSlugFont("./data/fonts/zapfino.slug");
    slug_font_handle_ = gos_LoadSlugFont("./data/fonts/terminus.slug");
    assert(slug_font_handle_);
	state_ = Component::kInitialized;

    text_size_ = 24;
}

void GameTextComp::DeinitRenderResources() {
    gos_DeleteFont(font_handle_);
    state_ = Component::kUninitialized;
}

void GameTextComp::AddRenderPackets(struct RenderFrameContext * rfc) const {

    uint32_t i = (int)(intensity_ * 255.0f + 0.5f);
    //               b        g         r       a
    uint32_t color = i | (i<<8) | (i<<16) | 0xFF000000;

    char pposbuf[32];
    vec4 ppos = rfc->proj_ * rfc->view_ * vec4(GetPosition(), 1);
    // TODO: using viewport here might not work if we render to some pass which uses different WxH, need to use per pass current viewport
    vec2 screenpos = (vec2(ppos.x/ppos.w, ppos.y/ppos.w)*vec2(0.5f, 0.5f) + vec2(0.5f))*vec2(rfc->viewport_.z, rfc->viewport_.w);
    snprintf(pposbuf, sizeof(pposbuf), "%.2f, %.2f, %.2f %.2f\n", screenpos.x, screenpos.y, ppos.z, ppos.w);

    uint64_t game_ticks = TimerGetGameTime();
    float game_time = (float)timing::ticks2ms(game_ticks)/1000.0f;
char buf[256]; sprintf(buf, "%.2f GT:%.1fs %s", frame_time_ms_, game_time, pposbuf); rfc->rl_->addTextPacket(buf, font_handle_, color, 64, 10, 5); //rfc->rl_->addSlugTextPacket("Hello!", small_font_handle_, color, 24, screenpos.x, screenpos.y);
    //rfc->rl_->addSlugTextPacket("This is Slug!", slug_font_handle_, color, text_size_, screenpos.x, screenpos.y);
    rfc->rl_->addSlugTextPacket("Hello!KLMWNO", slug_font_handle_, color, text_size_, screenpos.x, screenpos.y);
}

void GameTextComp::Initialize() {
    intensity_ = 0.75f;
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

PROPERTY_LIST_BEGIN_DERIVED(GameTextComp, Component)
    PROPERTY_FLOAT(intensity_, "intensity", 0, 0, 1, 0.01f);
    PROPERTY_INT(text_size_,"text size", 0, 1);
    PROPERTY_READONLY_TEXT("text", [](const GameTextComp& c) { return c.text; });
PROPERTY_LIST_END()



//------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------


void EnemyTextComp::InitRenderResources() {

    font_handle_ = gos_LoadFont("./data/fonts/roboto_medium_24");
    text_size_ = 24;
    assert(font_handle_);
	state_ = Component::kInitialized;
}

void EnemyTextComp::DeinitRenderResources() {
    gos_DeleteFont(font_handle_);
    state_ = Component::kUninitialized;
}

void EnemyTextComp::AddRenderPackets(struct RenderFrameContext * rfc) const {

    uint32_t i = (int)(intensity_ * 255.0f + 0.5f);
    //               b        g         r       a 
    uint32_t color = i | (i<<8) | (i<<16) | 0xFF000000;

    if(colour_) {
        color = colour_;
    }

    vec4 ppos = rfc->proj_ * rfc->view_ * vec4(GetPosition(), 1);
    // TODO: using viewport here might not work if we render to some pass which uses different WxH, need to use per pass current viewport
    vec2 screenpos = (vec2(ppos.x/ppos.w, ppos.y/ppos.w)*vec2(0.5f, +0.5f) + vec2(0.5f))*vec2(rfc->viewport_.z, rfc->viewport_.w);

    //char buf[256];
    //sprintf(buf, "[%s]", text);

    rfc->rl_->addTextPacket(text, font_handle_, color, 24, screenpos.x, screenpos.y);
    HGOSSLUGFONT font = gos_getSlugFont("./data/fonts/terminus.slug");
    if(font) {
       rfc->rl_->addSlugTextPacket(text, font, color, text_size_, screenpos.x, screenpos.y + 20);
    }
}

void EnemyTextComp::Initialize() { intensity_ = 0.5f; b_is_active_ = false; }
void EnemyTextComp::Deinitialize() { }

void EnemyTextComp::UpdateComponent(float dt) {
    // TODO: add game time (does not grow when game is paused)
    double time_s = 0.001*timing::ticks2ms(timing::gettickcount());
    intensity_ = saturate(0.25f*sin(time_s*0.5f*M_PI/180.0f) + 0.75f);
}

void EnemyTextComp::SetText(const char* t) {
    int count = min(strlen(t), sizeof(text)-1);
    memmove(text, t, count);
    text[count] = '\0';
}


PROPERTY_LIST_BEGIN_DERIVED(EnemyTextComp, Component)
    PROPERTY_FLOAT(intensity_, "intensity", 0, 0, 1, 0.01f);
    PROPERTY_UINT(colour_,"colour", 0, 0, 0xFFFFFFFF, 1);
    PROPERTY_INT(text_size_,"text size", 0, 1);
    PROPERTY_READONLY_TEXT("text", [](const EnemyTextComp& c) { return c.text; });
    PROPERTY_BOOL(b_is_active_, "is active");
PROPERTY_LIST_END()


