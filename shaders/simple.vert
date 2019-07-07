
layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 1) in vec3 normal;

uniform mat4 wvp_;
uniform mat4 world_;
uniform mat4 wvpshadow_;

out vec2 tc;
out vec3 norm;
out vec4 shadow_pos;

void main(void)
{
	gl_Position     = wvp_*vec4(pos, 1);
    shadow_pos      = wvpshadow_*vec4(pos, 1);

	tc = texcoord;
    norm = normal;
}

