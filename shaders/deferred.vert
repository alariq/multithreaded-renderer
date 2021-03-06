
layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 2) in vec3 normal;

uniform mat4 wvp_;

layout(location=0) out VsOut {
    vec2 texcoord;
    vec3 normal;
} o;

void main(void)
{
	gl_Position = wvp_*vec4(pos, 1);
	o.texcoord = texcoord;
    o.normal = normal;
}

