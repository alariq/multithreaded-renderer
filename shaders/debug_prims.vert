//#version 450

// material to draw debug lines, and other stuff
//

layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 uv;
layout(location = 2) in vec4 colour;

uniform mat4 wvp_;
uniform vec4 colour_;

out vec4 o_colour;
out vec2 o_uv;

void main(void)
{
	gl_Position = wvp_*vec4(pos, 1);
    o_colour = colour;
    o_uv = uv;
}

