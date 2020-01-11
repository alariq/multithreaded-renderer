#pragma once

#include "gameos.hpp"
#include <string>
class RenderMesh;

void initialize_res_man();
void finalize_res_man();

DWORD res_man_load_texture(const std::string &name);
RenderMesh *res_man_load_mesh(const std::string mesh_name);
