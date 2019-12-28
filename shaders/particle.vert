
layout(location = 0) in vec2 uv;

layout(location = 1) in float lifetime;
layout(location = 2) in vec3 pos;

uniform mat4 wvp_;

out vec2 o_uv;
out float o_lifetime;

void main(void)
{
    vec2 lpos       = uv*2.0 - 1.0;
	gl_Position     = wvp_*vec4(pos + vec3(lpos.xy, 0.0), 1);
    o_uv            = uv;
    o_lifetime      = lifetime;

}

