
layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 2) in vec3 normal;

layout(location = 3) in vec2 inst_pos;
layout(location = 4) in vec2 inst_vel;
layout(location = 5) in vec2 inst_force;
layout(location = 6) in float inst_density;
layout(location = 7) in float inst_pressure;
layout(location = 8) in float inst_flags;

uniform mat4 wvp_;

layout(location=0) out VsOut {
    vec2 texcoord;
    vec3 normal;
    vec2 force;
    float density;
    float pressure;
    float flags;
} o;

void main(void)
{
	gl_Position = wvp_*vec4(pos * vec3(0.1,0.1,0) + vec3(inst_pos.x, inst_pos.y, 0.0), 1);
	o.texcoord = texcoord;
    o.normal = normal;
    o.force = inst_force;
    o.density = inst_density;
    o.pressure = inst_pressure;
    o.flags = inst_flags;
}

