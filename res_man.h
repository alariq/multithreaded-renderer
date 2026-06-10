#pragma once

#include "engine/utils/my_types.h"
#include <string>

struct RenderMesh;
struct StaticMesh;

struct TextureRes;
typedef TextureRes* TextureHandle;

void initialize_res_man();
void finalize_res_man();
void res_man_update();

StaticMesh *res_man_load_mesh2(const std::string& mesh_name);
void res_man_release_mesh2(struct StaticMesh* mesh);

TextureHandle res_man_load_texture2(const std::string& name);

void res_man_schedule_rt_requests(struct RenderFrameContext* rfc);


u32 texture_res_get_gpu_id(TextureHandle h);
