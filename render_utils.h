#pragma once

#include "utils/vec.h"
#include <stdint.h>

void add_debug_mesh(struct RenderFrameContext *rfc, const struct RenderMesh *mesh, const mat4 &mat,
						   const vec4 &color, uint32_t selection_id = 0);

void add_debug_sphere_constant_size(struct RenderFrameContext *rfc, const vec3& pos, float no_scale_distance, const vec4& color);

// no_scale_distance - distance at which desired object will be drawn with no scale applied
void add_debug_mesh_constant_size(struct RenderFrameContext *rfc, const struct RenderMesh *mesh,
										 const vec4 &color, const mat4 &tr_m,
                                         const float no_scale_distance = .1f,
										 uint32_t selection_id = 0);

// only useful for 2d screen aligned objects
// pixels - approx amount of pixels object should have on the screen
void add_debug_mesh_constant_size_px(struct RenderFrameContext *rfc, const struct RenderMesh *mesh,
										 const vec4 &color, const mat4 &tr_m,
                                         const uint32_t pixels,
										 uint32_t selection_id = 0);

