#ifdef PLATFORM_WINDOWS
#define NOMINMAX
#include <windows.h>
#endif
#include <cassert>
#include <cstring>
#include <GL/glew.h>
#include "utils/shader_builder.h"
#include "utils/gl_utils.h"
#include "utils/vec.h"
#include "utils/camera.h"
#include "utils/gl_render_constants.h"
#include "utils/Image.h"

// maybe just 
// header: extern uint32_t TF_R8
// cpp: TF_R8 = GL_R8 
// ?
//
const GLint textureFormats[TF_COUNT] = {
    0,

    GL_RGB,
    GL_RGBA,

    GL_RED,
    GL_RG,
    GL_RGB,
    GL_RGBA,

    GL_RED,
    GL_RG,
    GL_RGB,
    GL_RGBA,

    GL_RED_INTEGER,
    GL_RG_INTEGER,
    GL_RGB_INTEGER,
    GL_RGBA_INTEGER,

    GL_RED_INTEGER,
    GL_RG_INTEGER,
    GL_RGB_INTEGER,
    GL_RGBA_INTEGER,

    GL_DEPTH_COMPONENT,
    GL_DEPTH_COMPONENT,
    GL_DEPTH_STENCIL,
    GL_DEPTH_STENCIL
};

const GLint textureInternalFormats[TF_COUNT] = {
    0,

    GL_SRGB8,
    GL_SRGB8_ALPHA8,

    GL_R8,
    GL_RG8,
    GL_RGB8,
    GL_RGBA8,

    GL_R32F,
    GL_RG32F,
    GL_RGB32F,
    GL_RGBA32F,

	GL_R16UI,
	GL_RG16UI,
	GL_RGB16UI,
	GL_RGBA16UI,

	GL_R32UI,
	GL_RG32UI,
	GL_RGB32UI,
	GL_RGBA32UI,

    GL_DEPTH_COMPONENT16,
    GL_DEPTH_COMPONENT32,
    GL_DEPTH24_STENCIL8,
    GL_DEPTH32F_STENCIL8
};


const int textureFormatNumChannels[TF_COUNT] = {
    0,
    3, 4,
    1, 2, 3, 4,
	1, 2, 3, 4,
	1, 2, 3, 4,
	1, 2, 3, 4,
    1, 1, 1, 1
};

const GLint textureFormatChannelType[TF_COUNT] = {
    0,
    GL_UNSIGNED_BYTE, GL_UNSIGNED_BYTE,
    GL_UNSIGNED_BYTE, GL_UNSIGNED_BYTE, GL_UNSIGNED_BYTE, GL_UNSIGNED_BYTE,
    GL_FLOAT, GL_FLOAT, GL_FLOAT, GL_FLOAT,
	GL_UNSIGNED_SHORT,GL_UNSIGNED_SHORT,GL_UNSIGNED_SHORT,GL_UNSIGNED_SHORT,
	GL_UNSIGNED_INT,GL_UNSIGNED_INT,GL_UNSIGNED_INT,GL_UNSIGNED_INT,
    // not much sense for depth formats...
    GL_HALF_FLOAT, GL_FLOAT, GL_UNSIGNED_INT, GL_FLOAT
};

const static uint32_t textureFormatChannelSize[TF_COUNT] = {
    0,
    1, 1,
    1, 1, 1, 1,
    4, 4, 4, 4,
	2, 2, 2, 2,
	4, 4, 4, 4,
    2, 4, 4, 4 // not much sense for depth/depth-stencil
};

uint32_t
getTexFormatPixelSize(TexFormat fmt) {

    assert(sizeof(textureFormatChannelSize)/sizeof(textureFormatChannelSize[0]) == TF_COUNT);
    assert(sizeof(textureFormatNumChannels)/sizeof(textureFormatNumChannels[0]) == TF_COUNT);

    return textureFormatChannelSize[fmt] * textureFormatNumChannels[fmt];
}

// must be in sync with TexType
const GLuint textureType[TT_COUNT] = {
    0,
    GL_TEXTURE_1D,
    GL_TEXTURE_1D_ARRAY,
    GL_TEXTURE_2D,
    GL_TEXTURE_2D_ARRAY,
    GL_TEXTURE_2D_MULTISAMPLE,
    GL_TEXTURE_2D_MULTISAMPLE_ARRAY,
    GL_TEXTURE_3D,
    GL_TEXTURE_CUBE_MAP,
    GL_TEXTURE_CUBE_MAP_ARRAY,
    GL_TEXTURE_RECTANGLE
};

GLenum translateTexType(TexType tt) {
    assert(tt > TT_NONE && tt < TT_COUNT);
    return textureType[tt];
}

const GLint textureAddressMode[TAM_COUNT] = {
    0,
    GL_CLAMP_TO_EDGE,
    GL_REPEAT
};

GLint getTextureAddressMode(TexAddressMode address_mode) {
    assert(address_mode > TAM_NONE && address_mode < TAM_COUNT);
    return textureAddressMode[address_mode];
}

const GLint textureFilterMode[TFM_COUNT] = {
    0,
    GL_NEAREST,
    GL_LINEAR,
    GL_NEAREST_MIPMAP_NEAREST,
    GL_NEAREST_MIPMAP_LINEAR,
    GL_LINEAR_MIPMAP_NEAREST,
    GL_LINEAR_MIPMAP_LINEAR,
};

GLint getTextureFilterMode(TexFilterMode filter_mode) {
    assert(filter_mode > TFM_NONE && filter_mode < TFM_COUNT);
    return textureFilterMode[filter_mode];
}

void destroyTexture(Texture* tex)
{
    assert(tex);

    if(tex->isValid())
        glDeleteTextures(1, &tex->id);
}

bool isImageTexture(TexFormat f) {
    return f >= TF_SRGB8 && f <=TF_RGBA8;
}

Texture create2DTexture(TexType type, TexFormat fmt, int w, int h, const uint8_t* texdata)
{
    // probably use functions for render target creation
    assert(fmt < TF_DEPTH16F &&
           "Shoud not create depth textures using this function");

    GLenum gl_target = translateTexType(type);
    GLenum gl_format = textureFormats[fmt];
    GLint gl_internal_format = textureInternalFormats[fmt];
    GLenum gl_channel_type = textureFormatChannelType[fmt];

    GLuint texID;
	glGenTextures(1, &texID);
	glBindTexture(gl_target , texID);

	glTexParameteri(gl_target, GL_TEXTURE_WRAP_S, type == TT_RECTANGLE ? GL_CLAMP : GL_REPEAT);
	glTexParameteri(gl_target, GL_TEXTURE_WRAP_T, type == TT_RECTANGLE ? GL_CLAMP : GL_REPEAT);
	glTexParameteri(gl_target, GL_TEXTURE_MAG_FILTER, type == TT_RECTANGLE ? GL_NEAREST : GL_LINEAR);
	glTexParameteri(gl_target, GL_TEXTURE_MIN_FILTER, type == TT_RECTANGLE ? GL_NEAREST : GL_LINEAR_MIPMAP_LINEAR);

    glTexImage2D(gl_target, 0, gl_internal_format, w, h, 0, gl_format, gl_channel_type, texdata);

	Texture t;
	t.id = texID;
	t.w = w;
	t.h = h;
    t.mips = 1;
	t.fmt = fmt;
    t.type = type;
    t.gl_internal_format = gl_internal_format;
    
    return t;

}

void generateMipmaps(Texture* t) {
#if 0
    int num_mips = ... // funcion parameter
    int calced_mips = 0;
    int size = max(w, h);
    while(size>0) {
        calced_mips++;
        size = size >> 1;
    }

    if(num_mips == -1) {
        num_mips = calced_mips;
    } else {
        num_mips = min(num_mips, calced_mips);
    }


    assert(num_mips = 0 || texdata);

    int my_w = w;
    int my_h = h;
    int offset = 0;

    for(int mip=0; mip < texdata_mips; ++mip) {
        glTexImage2D(gl_target, mip, gl_internal_format, my_w, my_h, 0, gl_format, gl_channel_type, texdata + offset);
        CHECK_GL_ERROR;
        offset += my_w * my_h;
        my_w = my_w >> 1;
        my_h = my_h >> 1;
    }

    if(texdata && num_mips > 1) {
        glGenerateMipmap(GL_TEXTURE_2D);
    }
#endif

    assert(t->type != TT_RECTANGLE);

    GLenum gl_target = translateTexType(t->type);
	glBindTexture(gl_target , t->id);
    glGenerateMipmap(gl_target);
    glBindTexture(gl_target, 0);
    t->mips = -1; // full chain
}

Texture createDynamicTexture(int w, int h, TexFormat fmt)
{
	GLuint texID;
	glGenTextures(1, &texID);
	glBindTexture(GL_TEXTURE_2D, texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, textureInternalFormats[fmt], 
            w, h, 0, textureFormats[fmt], textureFormatChannelType[fmt], NULL);
    CHECK_GL_ERROR

	Texture t;
	t.id = texID;
	t.w = w;
	t.h = h;
	t.fmt = fmt;
    t.type = TT_2D;
    t.gl_internal_format = (GLenum)-1;

	return t;
}

Texture create3DTextureF(int w, int h, int depth)
{
	GLuint texID;
	glGenTextures(1, &texID);
	glBindTexture(GL_TEXTURE_3D, texID);

	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, w, h, depth, 0, GL_RED, GL_FLOAT, NULL);

	Texture t;
	t.id = texID;
	t.w = w;
	t.h = h;
	t.depth = depth;
	t.gl_internal_format = GL_RED;
    t.type = TT_3D;
    t.fmt = TF_R32F;

	return t;
}

void updateTexture(const Texture& t, void* pdata) {

    assert(t.type == TT_2D);
    assert(t.fmt != TF_NONE && "t.format is deprecated");

	glBindTexture(GL_TEXTURE_2D, t.id);

    GLenum fmt = textureFormats[t.fmt];
    GLenum ch_type = textureFormatChannelType[t.fmt];
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, t.w, t.h, fmt, ch_type, pdata);
    CHECK_GL_ERROR

	glBindTexture(GL_TEXTURE_2D, 0);
}

void setSamplerParams(TexType tt, TexAddressMode address_mode, TexFilterMode filter) {

    GLuint tex_type = translateTexType(tt);

    GLint tam = getTextureAddressMode(address_mode);
    GLint tfm = getTextureFilterMode(filter);

	glTexParameteri(tex_type, GL_TEXTURE_WRAP_S, tam);
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_T, tam);
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_R, tam);

	glTexParameteri(tex_type, GL_TEXTURE_MAG_FILTER, tfm);
	glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, tfm);

}

void getTextureData(const Texture& t, int lod, unsigned char* poutdata, TexFormat format/*= TF_COUNT*/) {

    glBindTexture(GL_TEXTURE_2D, t.id);
    GLenum fmt = (format == TF_COUNT) ? textureFormats[t.fmt] : textureFormats[format];
    GLenum ch_type = textureFormatChannelType[t.fmt];
    glGetTexImage(GL_TEXTURE_2D, lod, fmt, ch_type, poutdata);
    CHECK_GL_ERROR
    glBindTexture(GL_TEXTURE_2D, 0);
}

void draw_quad(float x0, float y0, float x1, float y1)
{
	glBegin(GL_QUADS);
	
	glTexCoord2f(0,0);
	glVertex2f(x0,y0);
		
	glTexCoord2f(1,0);
	glVertex2f(x1, y0);
		
	glTexCoord2f(1,1);
	glVertex2f(x1, y1);
		
	glTexCoord2f(0,1);
	glVertex2f(x0, y1);

	glEnd();
}


void applyTexture(glsl_program* program, int unit, const char* name, GLuint texid)
{
    assert(program->samplers_.count( name ));

	if(program->samplers_.count( name ))
	{
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, texid);
		glUniform1i(program->samplers_[ name ]->index_, unit);
        CHECK_GL_ERROR
	}
}

void draw_in_2d(int w, int h, render_func_t prenderfunc, void* puserdata)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
    mat4 ortho_proj = orthoMatrix(0, (float)w, (float)h, 0, -1.0f, 1.0f, false);
	//gluOrtho2D(0, w, 0, h);
    glLoadTransposeMatrixf((const float*)ortho_proj);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	prenderfunc(w, h, puserdata);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


GLuint makeBuffer(GLenum target, const GLvoid* buffer_data, GLsizei buffer_size, GLenum type)
{
	GLuint buffer;
	glGenBuffers(1, &buffer);
    CHECK_GL_ERROR
	glBindBuffer(target, buffer);
    CHECK_GL_ERROR
	glBufferData(target, buffer_size, buffer_data, type);
    CHECK_GL_ERROR
	glBindBuffer(target, 0);
    CHECK_GL_ERROR
	return buffer;
}

void updateBuffer(GLuint buf, GLenum target, const GLvoid* buffer_data, GLsizei buffer_size)
{
	assert(buf && buffer_data);
	glBindBuffer(target, buf);
    CHECK_GL_ERROR
	glBufferSubData(target, 0, buffer_size, buffer_data);
    CHECK_GL_ERROR
	glBindBuffer(target, 0);
    CHECK_GL_ERROR
}


void updateBuffer(GLuint buf, GLenum target, const GLvoid* buffer_data, GLsizei buffer_size, GLenum type)
{
	assert(buf && buffer_data);
	glBindBuffer(target, buf);
    CHECK_GL_ERROR
	glBufferData(target, buffer_size, buffer_data, type);
    CHECK_GL_ERROR
	glBindBuffer(target, 0);
    CHECK_GL_ERROR
}



///////////////////////////////////////////////////////////////////////////////
// draw a textured cube with GL_TRIANGLES
///////////////////////////////////////////////////////////////////////////////
void draw_textured_cube(GLuint textureId)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureId);

	glColor4f(1, 1, 1, 1);
	glBegin(GL_TRIANGLES);
	// front faces
	glNormal3f(0,0,1);
	// face v0-v1-v2
	glTexCoord2f(1,1);  glVertex3f(1,1,1);
	glTexCoord2f(0,1);  glVertex3f(-1,1,1);
	glTexCoord2f(0,0);  glVertex3f(-1,-1,1);
	// face v2-v3-v0
	glTexCoord2f(0,0);  glVertex3f(-1,-1,1);
	glTexCoord2f(1,0);  glVertex3f(1,-1,1);
	glTexCoord2f(1,1);  glVertex3f(1,1,1);

	// right faces
	glNormal3f(1,0,0);
	// face v0-v3-v4
	glTexCoord2f(0,1);  glVertex3f(1,1,1);
	glTexCoord2f(0,0);  glVertex3f(1,-1,1);
	glTexCoord2f(1,0);  glVertex3f(1,-1,-1);
	// face v4-v5-v0
	glTexCoord2f(1,0);  glVertex3f(1,-1,-1);
	glTexCoord2f(1,1);  glVertex3f(1,1,-1);
	glTexCoord2f(0,1);  glVertex3f(1,1,1);

	// top faces
	glNormal3f(0,1,0);
	// face v0-v5-v6
	glTexCoord2f(1,0);  glVertex3f(1,1,1);
	glTexCoord2f(1,1);  glVertex3f(1,1,-1);
	glTexCoord2f(0,1);  glVertex3f(-1,1,-1);
	// face v6-v1-v0
	glTexCoord2f(0,1);  glVertex3f(-1,1,-1);
	glTexCoord2f(0,0);  glVertex3f(-1,1,1);
	glTexCoord2f(1,0);  glVertex3f(1,1,1);

	// left faces
	glNormal3f(-1,0,0);
	// face  v1-v6-v7
	glTexCoord2f(1,1);  glVertex3f(-1,1,1);
	glTexCoord2f(0,1);  glVertex3f(-1,1,-1);
	glTexCoord2f(0,0);  glVertex3f(-1,-1,-1);
	// face v7-v2-v1
	glTexCoord2f(0,0);  glVertex3f(-1,-1,-1);
	glTexCoord2f(1,0);  glVertex3f(-1,-1,1);
	glTexCoord2f(1,1);  glVertex3f(-1,1,1);

	// bottom faces
	glNormal3f(0,-1,0);
	// face v7-v4-v3
	glTexCoord2f(0,0);  glVertex3f(-1,-1,-1);
	glTexCoord2f(1,0);  glVertex3f(1,-1,-1);
	glTexCoord2f(1,1);  glVertex3f(1,-1,1);
	// face v3-v2-v7
	glTexCoord2f(1,1);  glVertex3f(1,-1,1);
	glTexCoord2f(0,1);  glVertex3f(-1,-1,1);
	glTexCoord2f(0,0);  glVertex3f(-1,-1,-1);

	// back faces
	glNormal3f(0,0,-1);
	// face v4-v7-v6
	glTexCoord2f(0,0);  glVertex3f(1,-1,-1);
	glTexCoord2f(1,0);  glVertex3f(-1,-1,-1);
	glTexCoord2f(1,1);  glVertex3f(-1,1,-1);
	// face v6-v5-v4
	glTexCoord2f(1,1);  glVertex3f(-1,1,-1);
	glTexCoord2f(0,1);  glVertex3f(1,1,-1);
	glTexCoord2f(0,0);  glVertex3f(1,-1,-1);
	glEnd();

	if(textureId)
		glBindTexture(GL_TEXTURE_2D, 0);
}

/*
glMesh<SIMPLE_VERTEX_PTN>* make_mesh_from_file(const char* filepath)
{
	assert(filepath);
	COMMON_MESH* pcomesh = load_mesh_from_binary(filepath);
	assert(pcomesh);

	if(pcomesh->va.stride != sizeof(SIMPLE_VERTEX_PTN))
		return 0;
	if(pcomesh->vd.decl.size() !=3)
		return 0;
	if(pcomesh->vd.decl[0].comp_type!=TYPE_FLOAT || pcomesh->vd.decl[0].num_comp!=3)
		return 0;
	if(pcomesh->vd.decl[1].comp_type!=TYPE_FLOAT || pcomesh->vd.decl[1].num_comp!=2)
		return 0;
	if(pcomesh->vd.decl[2].comp_type!=TYPE_FLOAT || pcomesh->vd.decl[2].num_comp!=3)
		return 0;
	
	glMesh<SIMPLE_VERTEX_PTN>* pmesh = new glMesh<SIMPLE_VERTEX_PTN>();
	pmesh->num_indices_ = 0;
	pmesh->num_vertices_ = pcomesh->va.num_vertices;
	pmesh->prim_type_ = GL_TRIANGLES;
	pmesh->pvertices_ = new SIMPLE_VERTEX_PTN[pcomesh->va.num_vertices];
	memcpy(pmesh->pvertices_, pcomesh->va.data, pcomesh->va.num_vertices*sizeof(SIMPLE_VERTEX_PTN));

	delete pcomesh;

	pmesh->gen_hw(GL_STATIC_DRAW);
	return pmesh;
}
*/
