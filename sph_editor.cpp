#include "sph_editor.h"
#include "sph_object.h"
#include "sph.h"
#include "scene.h"
#include "editor.h"
#include "engine/gameos.hpp"
#include "engine/utils/vec.h"
#include "engine/utils/camera.h"
#include "engine/utils/camera_utils.h"

static int g_sph_editor_id = -1;

GameObject* sph_editor_update(camera* cam, float dt, GameObject* sel_go) {

    SPHSimulation* sim = sph_get_simulation();

    int XDelta, YDelta, WheelDelta;
    float XPos, YPos;
    DWORD buttonsPressed;
    gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);

    if(gos_GetKeyStatus(KEY_A) == KEY_PRESSED) {
	    vec2 mouse_screen_pos = 2 * vec2(XPos, 1 - YPos) - 1;
        vec3 dir = screen2world_vec(cam, mouse_screen_pos);
        vec3 int_pt = ray_plane_intersect(dir, cam->get_pos(), make_plane(vec3(0,0,1),vec3(0,0,0)));

        SPHBoundaryObject* bo = new SPHBoundaryObject(sim, vec2(1,1), false);
        auto tr = bo->GetComponent<TransformComponent>();
        tr->SetPosition(int_pt);
        scene_add_game_object(bo);
    }
    else if(gos_GetKeyStatus(KEY_DELETE) == KEY_PRESSED && sel_go) {
        scene_delete_game_object(sel_go);
        return nullptr;
    }

    return sel_go;
}

void sph_editor_render_update(struct RenderFrameContext* rfc) {

}

bool sph_editor_wants_activate() {
    if(gos_GetKeyStatus(KEY_LCONTROL) == KEY_HELD && gos_GetKeyStatus(KEY_S) == KEY_PRESSED) {
        return true;
    }
    return false;
}

const char* sph_editor_name() {
    return "sph editor";
}

void sph_editor_init() {
    UserEditorInterface uei;
    uei.update = sph_editor_update;
    uei.render_update = sph_editor_render_update;
    uei.wants_activate = sph_editor_wants_activate;
    uei.name = sph_editor_name;
    g_sph_editor_id = editor_register_user_editor(uei);
}

void sph_editor_deinit() {
    editor_unregister_user_editor(g_sph_editor_id);
    g_sph_editor_id = -1;
}

