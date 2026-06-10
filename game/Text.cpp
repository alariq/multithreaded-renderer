#include "../obj_model.h"
#include "engine/utils/timing.h"
#include "Text.h"
#include "Time.h"
#include "gameos.hpp"

class FontRenderProxy: public IRenderProxy {
    HGOSFONT3D font_handle_ = 0;
    HGOSSLUGFONT slug_font_handle_ = 0;
    std::string name_;
    bool b_is_slug_;
public:
    FontRenderProxy():font_handle_(0), slug_font_handle_(0),
        b_is_slug_(false), size_(0) {}

    std::string text_;
    int size_; // only for slug
    vec2 screenpos_;
    u32 color_;

    // TODO: font should also be resource and split on main + rt parts
    void SetFontName(const std::string& name, bool b_is_slug) {
        if(name_ != name) {
            if(font_handle_) {
                //FIXME: cannot delet fonts at the moment, because they could be used
                //by others and they are not ref counted at the moment
                gos_DeleteFont(font_handle_);
                font_handle_ = 0;
            }
            if(slug_font_handle_) {
                //FIXME: cannot delet fonts at the moment, because they could be used
                //by others and they are not ref counted at the moment
                gos_DeleteSlugFont(slug_font_handle_);
                slug_font_handle_ = 0;
            }

            b_is_slug_ = b_is_slug;
            if(b_is_slug) {
                slug_font_handle_ = gos_LoadSlugFont(name.c_str());
            } else {
                font_handle_ = gos_LoadFont(name.c_str());
            }
            name_ = name;
        }
    }

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) override {
        if(b_is_slug_ && slug_font_handle_) {
            rfc->rl_->addSlugTextPacket(text_.c_str(), slug_font_handle_, color_, size_, screenpos_.x, screenpos_.y);
        } else if(!b_is_slug_ && font_handle_) {
            rfc->rl_->addTextPacket(text_.c_str(), font_handle_, color_, 24, screenpos_.x, screenpos_.y);
        }
    }

    virtual void Initialize(struct RenderFrameContext* ) override {

    }

    virtual void Deinitialize(struct RenderFrameContext* ) override {
        if(font_handle_)
            gos_DeleteFont(font_handle_);
        if(slug_font_handle_)
            gos_DeleteSlugFont(slug_font_handle_);
    }

};

IRenderProxy* GameTextComp::CreateRenderProxy() {
    proxy_ = new FontRenderProxy();
    return proxy_;
}

void GameTextComp::DestroyRenderProxy(struct RenderFrameContext* rfc) {
    ScheduleRenderCommand(rfc, [proxy = proxy_]() {
        delete proxy;
    });
    proxy_ = nullptr;
}
IRenderProxy* GameTextComp::GetRenderProxy() {
    gosASSERT(proxy_);
    return proxy_;
}

void GameTextComp::RenderUpdateComponent(struct RenderFrameContext * rfc) {

    uint32_t i = (int)(intensity_ * 255.0f + 0.5f);
    //               b        g         r       a
    uint32_t color = i | (i<<8) | (i<<16) | 0xFF000000;

    char pposbuf[32];
    vec4 ppos = rfc->proj_ * rfc->view_ * vec4(GetPosition(), 1);
    // TODO: using viewport here might not work if we render to some pass which uses different WxH, need to use per pass current viewport
    vec2 screenpos = (vec2(ppos.x/ppos.w, ppos.y/ppos.w)*vec2(0.5f, 0.5f) + vec2(0.5f))*vec2(rfc->viewport_.z, rfc->viewport_.w);
    snprintf(pposbuf, sizeof(pposbuf), "%.2f, %.2f, %.2f %.2f\n", screenpos.x, screenpos.y, ppos.z, ppos.w);

    //FIXME: TODO: font render proxy is cool and so on, but we can have only one, actually would be better to move font to resource as well, 
    // so we could create it on main thread and then just use simple addTextPacket() to draw whatever we want
#if 0
    uint64_t game_ticks = TimerGetGameTime();
    float game_time = (float)timing::ticks2ms(game_ticks)/1000.0f;
    char buf[256];
    sprintf(buf, "%.2f GT:%.1fs %s", frame_time_ms_, game_time, pposbuf); rfc->rl_->addTextPacket(buf, font_handle_, color, 64, 10, 5);
#endif

    ScheduleRenderCommand(rfc, [proxy = proxy_, color, size = text_size_, text = text, screenpos, name = slug_font_name_]() {
        proxy->size_ = size;
        proxy->screenpos_ = screenpos;
        proxy->color_ = color;
        proxy->text_ = text;
        proxy->SetFontName(name, true);
    });
}

void GameTextComp::Initialize() {
    intensity_ = 0.75f;
    frame_time_ms_ = 0.0f;
    text_size_ = 24;
    strcpy(text, "Hello!KLMWNO");

    //"./data/fonts/zapfino.slug"
    //"./data/fonts/terminus.slug"
    //"./data/fonts/OpenDyslexic.slug"
    font_name_ = "./data/fonts/roboto_medium_24";
    slug_font_name_ = "./data/fonts/roboto_medium.slug";

    TransformComponent::Initialize();
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


void EnemyTextComp::Initialize() {
    text_size_ = 24;
    intensity_ = 0.5f;
    b_is_active_ = false; 

    TransformComponent::Initialize();
}

IRenderProxy* EnemyTextComp::CreateRenderProxy() {
    proxy_ = new FontRenderProxy();
    return proxy_;
}

void EnemyTextComp::DestroyRenderProxy(struct RenderFrameContext* rfc) {
    ScheduleRenderCommand(rfc, [proxy = proxy_]() {
        delete proxy;
    });
    proxy_ = nullptr;
}
IRenderProxy* EnemyTextComp::GetRenderProxy() {
    gosASSERT(proxy_);
    return proxy_;
}

void EnemyTextComp::RenderUpdateComponent(struct RenderFrameContext * rfc) {

    uint32_t i = (int)(intensity_ * 255.0f + 0.5f);
    //               b        g         r       a 
    uint32_t color = i | (i<<8) | (i<<16) | 0xFF000000;

    if(colour_) {
        color = colour_;
    }

    vec4 ppos = rfc->proj_ * rfc->view_ * vec4(GetPosition(), 1);
    // TODO: using viewport here might not work if we render to some pass which uses different WxH, need to use per pass current viewport
    vec2 screenpos = (vec2(ppos.x/ppos.w, ppos.y/ppos.w)*vec2(0.5f, +0.5f) + vec2(0.5f))*vec2(rfc->viewport_.z, rfc->viewport_.w);
    // TODO: FIXME: as soon fong will be a real resource, I will be able to just use this:
    // no render proxies will be needed
    //rfc->rl_->addTextPacket(text, font_handle_, color, 24, screenpos.x, screenpos.y);

    ScheduleRenderCommand(rfc, [proxy = proxy_, color, size = text_size_, text = text, screenpos]() {
        proxy->size_ = size;
        proxy->screenpos_ = screenpos;
        proxy->color_ = color;
        proxy->text_ = text;
        proxy->SetFontName("./data/fonts/roboto_medium_24", false);
    });

}

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


