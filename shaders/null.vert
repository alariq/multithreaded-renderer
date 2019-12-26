//#version 450
layout(location = 0) in vec3 pos;

uniform mat4 wvp_;

void main(void)
{
	gl_Position = wvp_*vec4(pos, 1);
}

