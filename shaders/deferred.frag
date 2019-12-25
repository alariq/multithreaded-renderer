#define PREC highp

layout (location=0) out PREC vec4 g_buffer_albedo;
layout (location=1) out PREC vec4 g_buffer_normal;

layout(location=0) in PsIn {
    PREC vec2 texcoord;
    PREC vec3 normal;
} Input;


uniform sampler2D tex1;

void main(void)
{
	PREC vec4 albedo = texture2D(tex1, vec2(Input.texcoord.x, 1-Input.texcoord.y));

    g_buffer_albedo = albedo;
    g_buffer_normal.rgb = Input.normal*0.5 + 0.5;
    //g_buffer_albedo.rgb = Input.normal*0.5 + 0.5;
    g_buffer_normal.w = 1.0;
}

