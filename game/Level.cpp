#include "Level.h"
#include "Time.h"
#include "Billboard.h"
#include "game/Text.h"
#include "engine/profiler/profiler.h"
#include "engine/utils/timing.h"
#include "engine/utils/camera.h"
#include "engine/utils/imgui_property_list.h"
#include "gameos.hpp"

#include <cctype>

MainShip* g_main_ship = nullptr;

camera ship_cam;
float ship_cam_distance = 10.0f;
struct ShipCam {
    struct State { vec3 pos; quaternion qrot;
    };

    State cur;
    State target;

    void init(vec3 pos, quaternion rot) {
        cur.pos = pos;
        cur.qrot = rot;

        target = cur;
    }

} ship_cam_state;

void update_ship_cam(MainShip* ship) {

	int XDelta, YDelta, WheelDelta;
    float XPos, YPos;
    DWORD buttonsPressed;
    gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);
    if(WheelDelta) {
        ship_cam_distance *= WheelDelta > 0 ? 1.1f : 0.9f;
    }

    // TODO: fix this mess
    extern ivec4 editor_get_3dview_rect();
    ivec4 scene_rect = editor_get_3dview_rect();

    vec3 ship_pos = ship->GetComponent<TransformComponent>()->GetPosition();
    ship_cam.set_projection(90.0f, scene_rect.z, scene_rect.w, 0.1f, 1000.0f);
    //ship_cam.lookat(ship_cam_distance*vec3(1,1,1),  ship_pos, vec3(0,1,0));
    quaternion rot = ship->GetComponent<TransformComponent>()->GetRotation();
    vec3 elevation = vec3(0,1.5f,0);

    ship_cam_state.target.pos = ship_pos + elevation;
    ship_cam_state.target.qrot = rot;

    const float dt = 0.016f;
    // Math In Game Development Summit: Getting There in Style: Intro to Interpolation and Control Systems 
    // exponential decay, K = 2.0
    // https://theorangeduck.com/page/spring-roll-call
    float halflife_sec = .25f; // halflife = 0.693 / k (50% of error)
    float k = 0.693 / halflife_sec;
    const float alpha = 1.0f - std::exp(-dt * k);
    ship_cam_state.cur.pos = //ship_cam_state.target.pos;
        lerp(ship_cam_state.cur.pos, ship_cam_state.target.pos, alpha);
    ship_cam_state.cur.qrot = //ship_cam_state.target.qrot;
        slerp(ship_cam_state.cur.qrot, normalize(ship_cam_state.target.qrot), alpha);

    mat4 view = camera::make_lookat(
            ship_cam_state.cur.pos - ship_cam_distance*ship_cam_state.cur.qrot.axis2() + vec3(0, 0.2f, 0)*ship_cam_distance,
            ship_cam_state.cur.pos,
            ship_cam_state.cur.qrot.axis1());

    ship_cam.set_view(view);
}


void cam_updator_func() {
    update_ship_cam(g_main_ship); 
}

typedef void (*CamerUpdateFuncPtr)(void);
void SetCameraOverride(camera* cam, CamerUpdateFuncPtr update_func);


Level* Level::Create(const char* res) {

    static size_t obj_num = 0;
    Level* obj = new Level();

    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    //obj->AddComponent<FrustumComponent>();

    //TODO: move game text component from MainShip here
    //auto* TextComp = obj->AddComponent<GameTextComp>();

    // create checkpoints
    std::vector<CheckPoint> cps;

    float torus_scale = 1;
    vec3 curve_dir = vec3(1, 0, 1);
    float node_span = 20;
    vec3 curve_start = vec3(20, 10, 20);
    vec3 dp = vec3(5.0f, 2.0f, 5.0f);
    const int num_cps = 15;//4;

    // gen checkpoint positions
    // -----------------------------------------------------------------------------------
    for(int i=0;i<num_cps; i++) {
        vec3 pos = curve_start + node_span*i*curve_dir + random_vec(-dp, dp);
        cps.push_back({pos, torus_scale});
    }

    // create curve passing through them
    // -----------------------------------------------------------------------------------
    obj->mainCurve_.addPoint(cps[0].position + 0.25f*(cps[0].position - cps[1].position));
    for(int i=0;i<num_cps; i++) {
        obj->mainCurve_.addPoint(cps[i].position);
    }
    obj->mainCurve_.addPoint(cps[num_cps-1].position + 0.25f*(cps[num_cps-1].position - cps[num_cps-2].position));

    // create component from the curve
    obj->mainCurveComp_ = obj->AddComponent<C_Curve>();
    obj->mainCurveComp_->SetCurve(&obj->mainCurve_);

    // generate toruses along the main curve checkpoints
    // -----------------------------------------------------------------------------------
    const int num_nodes = (int)obj->mainCurve_.getNumNodes();
    for(int i=0;i<num_nodes; i++) {

        GameObject* go = MeshObject::Create("torus");
        TransformComponent* tc = go->GetComponent<TransformComponent>();

        float pos_t = (float)i;
        float der_t = (float)i;
        float dt = 0.01f*node_span;

        vec3 pos;
        vec3 right, up, fwd;

        // for the last point to not go over the end of the curve, just get value at the end of the prev segment
        if(i == num_nodes-1) {
            pos_t -= 0.01f;
            der_t -= 2*dt;
        }

        pos = obj->mainCurve_.getAt(pos_t);
        spline_get_basis_at(obj->mainCurve_, der_t, dt, right, up, fwd);

        mat3 m = mat3(right.x, right.y, right.z, up.x, up.y, up.z, fwd.x, fwd.y, fwd.z);
        m = transpose(m);
        //printf("m%d = %.2f %.2f %.2f \n%.2f %.2f %.2f\n%.2f %.2f %.2f", i,
         //   right.x, right.y, right.z, up.x, up.y, up.z, fwd.x, fwd.y, fwd.z);

        quaternion q = mat3_to_quat(m); // mat to quat produces inverted quat (expects transposed matrix input, TODO: rewrite)
        //printf("q: %.2f %.2f %.2f %.2f\n", q.x, q.y, q.z, q.w);
        //printf("q: %.2f %.2f %.2f %.2f\n", q.x, q.y, q.z, q.w);
        tc->SetRotation(q);
        tc->SetPosition(pos);
        tc->SetScale(vec3(torus_scale));

        scene_add_game_object(go);
    }

    obj->last_time_spawned_ = TimerGetGameTime();
    obj->spawn_interval_ms_ = 5000;
    obj->total_spawned_ = 0;

    // create player
    // -----------------------------------------------------------------------------------
    //g_main_ship = MainShip::Create("fighter1");
    MainShip* mainShip = MainShip::Create("paper_plane");
    TransformComponent* ftc = mainShip->GetComponent<TransformComponent>();
    //ftc->SetPosition(vec3(10, 4, 10));
    ftc->SetPosition(obj->mainCurve_.getAt(0));
    ftc->SetScale(vec3(.5f));
    mainShip->Initialize(cps, nullptr /* target */, obj->mainCurveComp_);
    scene_add_game_object(mainShip);

    // init main ship camera
    ship_cam.set_projection(45.0f, Environment.drawableWidth, Environment.drawableHeight, 0.1f, 1000.0f);
    ship_cam.set_view(mat4::identity());
    SetCameraOverride(&ship_cam, cam_updator_func);

    obj->mainShip = mainShip;
    g_main_ship  = obj->mainShip;


    // add some decorations
    auto bb = O_Billboard::Create("clouds");
    auto tr = bb->GetComponent<C_Billboard>();
    tr->SetScale(vec3(100, 100, 1));
    tr->SetPosition(vec3(500, 100, 500));
    scene_add_game_object(bb);

    return obj;
}

void Level::simulateFixedStep(double dt) {

    assert(mainShip);


    const auto* tc = mainShip->GetComponent<TransformComponent>();
    const vec3 pos = tc->GetPosition();
    const vec3 dir = tc->GetRotation().axis2();

    // this should not be inside fixed step as this is a constant related to it
    // though if all object will go in lock step then it should be inside and calculated every frame
    // but we actually only care about last position of main ship on the mainCurve (during last call to fixedStep), so could also leverat this fact
    SplineClosestPointResult res;
    {SCOPED_ZONE_N(GetClosest, 0);
    res = spline_get_closest_point(mainCurve_, pos, 50, 16);
    }
    const vec3 pos_on_spline  = res.point;
    vec3 fwd = normalize(mainCurve_.getDerivativeAt(res.t));
    vec3 up = vec3(0,1,0);
    vec3 right = cross(up, fwd);

    //orthonormalize_basis(fwd, up, right);

    float spawn_interval = spawn_interval_ms_ / (1.0f + 0.5f*max(1, int(total_spawned_/5)));

    uint64_t cur_gt = TimerGetGameTime();
    float spawn_dt = timing::ticks2ms(cur_gt - last_time_spawned_);
    if(spawn_dt > spawn_interval && enemies_.size() < 3) {
        Enemy* e = Enemy::Create("enemy1");
        e->GetComponent<TransformComponent>()->SetPosition(tc->GetPosition());
        scene_add_game_object(e);
        total_spawned_++;
        last_time_spawned_ = cur_gt;
        enemies_.push(e);
    }

    EnemyTargetCtx ctx = {
        .pos = pos,
        .dir = dir,

        .path_pos = pos_on_spline,
        .right = right,
        .up = up,
        .fwd = fwd 
    };

    ctx_ = ctx;

    for(int i=0;i<enemies_.size();) {
        enemies_[i]->UpdateTargetCtx(ctx);
        if(enemies_[i]->GetState().num_cycles > 0) {
            if(text_target_ == enemies_[i]) {
                text_target_->SetActiveTarget(false);
                text_target_ = nullptr;
            }
            enemies_[i]->Destroy();
            enemies_.remove_swap(i);
        } else {
            i++;
        }
    }

}

void Level::UpdateTextInput() {

    const DWORD key = gos_GetKey();
    if(key == 0) {
        return;
    }

    char c = (char)(key & 0xFF);
    if(c == 0) {
        return;
    }

    const char typed = (char)std::tolower((unsigned char)c);

    auto skip_spaces = [](const char* text) -> const char* {
        if(!text) {
            return nullptr;
        }
        while(*text == ' ') {
            ++text;
        }
        return text;
    };

    auto find_enemy_index = [this](Enemy* enemy) -> int {
        for(int i = 0; i < enemies_.size(); ++i) {
            if(enemies_[i] == enemy) {
                return i;
            }
        }
        return -1;
    };

    auto consume_char = [this, &find_enemy_index, &skip_spaces](Enemy* enemy) {
        EnemyTextComp* txt_comp = enemy->GetComponent<EnemyTextComp>();
        if(!txt_comp) {
            return;
        }

        const char* text = skip_spaces(txt_comp->GetText());
        if(!text || text[0] == '\0') {
            enemy->Destroy();
            int idx = find_enemy_index(enemy);
            assert(idx>=0);
            if(idx >= 0) {
                enemies_.remove_swap(idx);
            }
            if(text_target_ == enemy) {
                text_target_->SetActiveTarget(false);
                text_target_ = nullptr;
            }
            return;
        }

        const char* next = skip_spaces(text + 1);
        if(!next || next[0] == '\0') {
            enemy->Destroy();
            int idx = find_enemy_index(enemy);
            assert(idx>=0);
            if(idx >= 0) {
                enemies_.remove_swap(idx);
            }
            if(text_target_ == enemy) {
                text_target_->SetActiveTarget(false);
                text_target_ = nullptr;
            }
        } else {
            txt_comp->SetText(next);
        }
    };

    if(text_target_) {
        int target_idx = find_enemy_index(text_target_);
        assert(target_idx>=0);
        if(target_idx < 0) {
            text_target_ = nullptr;
        }
    }


    if(text_target_) {
        EnemyTextComp* txt_comp = text_target_->GetComponent<EnemyTextComp>();
        assert(txt_comp);

        const char* text = skip_spaces(txt_comp->GetText());
        if(!text || text[0] == '\0') {
            text_target_->SetActiveTarget(false);
            text_target_ = nullptr;
        } else {
            const char expected = (char)std::tolower((unsigned char)text[0]);
            if(expected == typed) {
                consume_char(text_target_);
            }
        }
    }

    if(!text_target_) {

        Enemy* best = nullptr;
        float best_dist_sq = 0.0f;

        vec3 ref_pos(0.0f);
        if(mainShip) {
            TransformComponent* tc = mainShip->GetComponent<TransformComponent>();
            if(tc) {
                ref_pos = tc->GetPosition();
            }
        }

        for(int i = 0; i < enemies_.size(); ++i) {
            Enemy* enemy = enemies_[i];
            EnemyTextComp* txt_comp = enemy->GetComponent<EnemyTextComp>();
            assert(txt_comp);

            const char* text = skip_spaces(txt_comp->GetText());
            if(!text || text[0] == '\0') {
                continue;
            }

            const char expected = (char)std::tolower((unsigned char)text[0]);
            if(expected != typed) {
                continue;
            }

            TransformComponent* etc = enemy->GetComponent<TransformComponent>();
            if(!etc) {
                if(!best) {
                    best = enemy;
                    best_dist_sq = 0.0f;
                }
                continue;
            }

            vec3 d = etc->GetPosition() - ref_pos;
            float dist_sq = dot(d, d);
            if(!best || dist_sq < best_dist_sq) {
                best = enemy;
                best_dist_sq = dist_sq;
            }
        }

        if(best) {
            text_target_ = best;
            text_target_->SetActiveTarget(true);
            consume_char(best);
        }
    }
}

void Level::Update(float dt) {

    // TODO: move to better place (this is only updated in game mode)
    static bool b_frustum_overriden = false;
    auto c_frustum = GetComponent<FrustumComponent>();
    if(c_frustum && gos_GetKeyStatus(KEY_F) == KEY_PRESSED && gos_GetKeyStatus(KEY_LCONTROL) == KEY_HELD)
    {
        const SceneViewInfo& svi = scene_get_view_info();
        if(b_frustum_overriden)
            c_frustum->OverrideView(nullptr);
        else
            c_frustum->OverrideView(&svi.view_mat_, &svi.inv_view_mat_, svi.fov_, 1, 10, svi.aspect_);

        b_frustum_overriden = !b_frustum_overriden;
    }

    UpdateTextInput();

    const double fixedDt = 0.016;
    const int maxSubSteps = 64;
    const int simSpeed = 1;
    const double maxAccumulated = fixedDt * maxSubSteps;
    accumulator = min(accumulator + dt*simSpeed, maxAccumulated);

    int steps = 0;
    while (accumulator >= fixedDt && steps < maxSubSteps) {
        simulateFixedStep(fixedDt);
        accumulator -= fixedDt;
        steps++;
    }
    assert(accumulator <= fixedDt);
}

void Level::AddRenderPackets(struct RenderFrameContext* rfc) const {

    vec3 pos = ctx_.path_pos;
    vec3 fwd = ctx_.fwd;
    vec3 up = ctx_.up;
    vec3 right = ctx_.right;

    RenderList* rl = rfc->rl_;
    rl->addDebugLine(pos, pos + 3*right, vec4(1, 0,0, 1));
    rl->addDebugLine(pos, pos + 3*up, vec4(0, 1,0, 1));
    rl->addDebugLine(pos, pos + 3*fwd, vec4(0, 0,1, 1));
}

PROPERTY_LIST_BEGIN_DERIVED(Level, GameObject)
    PROPERTY_INT(total_spawned_, "total enemies spawned", PropertyFlags::kPropertyFlagReadOnly);
    //PROPERTY_CHILD_PTR(mainShip , "MainShip");
PROPERTY_LIST_END()
