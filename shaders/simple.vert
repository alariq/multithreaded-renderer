//#version 450

layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texcoord;
layout(location = 2) in vec3 normal;

uniform mat4 wvp_;
uniform mat4 world_;
uniform mat4 world_normal_; // inverse transpose of a world matrix
uniform mat4 wvpshadow0_;
uniform mat4 wvpshadow1_;

out vec2 tc;
out vec3 norm;
out vec3 opos;
out vec4 shadow_pos0;
out vec4 shadow_pos1;

void main(void)
{
	gl_Position     = wvp_*vec4(pos, 1);
    shadow_pos0      = wvpshadow0_*vec4(pos, 1);
    shadow_pos1      = wvpshadow1_*vec4(pos, 1);

    opos = pos;
	tc = texcoord;
    //norm = (transpose(inverse(world_normal_)) * vec4(normal, 0.0)).xyz;
    norm = (world_normal_ * vec4(normal, 0)).xyz;
}

