layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 2) in vec3 normal;

uniform mat4 wvp_;

out vec2 o_pos;
out vec3 o_normal;

void main(void)
{
	gl_Position     = wvp_*vec4(pos, 1);
    o_normal = normal;
}

