
layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 1) in vec3 normal;

uniform mat4 wvp_;

out vec2 tc;
out vec3 norm;

void main(void)
{
	gl_Position     = wvp_*vec4(pos, 1);
	tc = texcoord;
    norm = normal;
}

