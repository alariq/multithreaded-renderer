//#version 450
layout(location = 0) in vec2 pos;
layout(location = 1) in vec2 uv;

uniform mat4 inv_proj_;

out vec2 o_pos;
out vec2 o_uv;
out vec3 o_viewray;


void main(void)
{
	gl_Position     = vec4(pos.xy, 0.9999, 1);
    o_pos = pos;
    o_uv = pos*0.5 + 0.5;

    vec3 pos_view_space = (inv_proj_ * vec4(pos.xy, 0.9990, 1)).xyz;
    o_viewray = vec3(pos_view_space.xy / pos_view_space.z, 1.0);

}

