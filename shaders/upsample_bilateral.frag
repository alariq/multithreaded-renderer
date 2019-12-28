#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex;
uniform sampler2D tex1;
uniform sampler2D tex2;

in PREC vec2 o_pos;
in PREC vec2 o_uv;

//TODO: actually upsample bilaterally!!!
void main(void)
{
	FragColor = texture2D(tex, o_uv);
}
