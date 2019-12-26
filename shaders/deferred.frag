//#version 450

#define PREC highp

layout (location=0) out PREC vec4 g_buffer_albedo;
layout (location=1) out PREC vec4 g_buffer_normal;

layout(location=0) in PsIn {
    PREC vec2 texcoord;
    PREC vec3 normal;
} Input;


uniform sampler2D tex1;
uniform mat4 wv_;

void main(void)
{
	PREC vec4 albedo = texture(tex1, vec2(Input.texcoord.x, 1-Input.texcoord.y));

    g_buffer_albedo = albedo;
    //TODO: optimize
    mat4 normal_wv = transpose(inverse(wv_));
    g_buffer_normal.rgb = 0.5*normalize((normal_wv * vec4(Input.normal, 0.0)).xyz) + 0.5;
    g_buffer_normal.w = 1.0;
}

