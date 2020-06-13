#ifndef __GL_UTILS_H__
#define __GL_UTILS_H__

#include <cassert>
#include <cstdio>
#include "utils/camera.h"
#include "utils/shader_builder.h"
#include "utils/render_constants.h"

#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + (bytes))

uint32_t vec4_to_uint32(const vec4& v);
vec4 uint32_to_vec4(uint32_t v);

struct Texture {
	Texture():id(0), w(0), h(0), depth(1), fmt_(TF_NONE) {}
    bool isValid() { return id > 0; }

	GLuint id;
	GLenum format;
	int w, h, depth;
    TexFormat fmt_;
    TexType type_;

};

uint32_t getTexFormatPixelSize(TexFormat fmt);

#define CHECK_GL_ERROR \
{ \
   	GLenum err = glGetError();\
	while(err != GL_NO_ERROR) \
	{ \
		printf("OpenGL Error: %s\n", ogl_get_error_code_str(err)); \
		printf("Location : %s : %d\n", __FILE__ , __LINE__); \
   	    err = glGetError();\
	}\
}

template<typename T>
const char* ogl_get_error_code_str(T input)
{
    int errorCode = (int)input;
    switch(errorCode)
    {
	case GL_NO_ERROR:
            return "GL_NO_ERROR";
        case GL_INVALID_ENUM:
            return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE:
            return "GL_INVALID_VALUE";               
        case GL_INVALID_OPERATION:
            return "GL_INVALID_OPERATION";           
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            return "GL_INVALID_FRAMEBUFFER_OPERATION";      
        case GL_OUT_OF_MEMORY:
            return "GL_OUT_OF_MEMORY";                    
        case GL_STACK_OVERFLOW:
            return "GL_STACK_OVERFLOW";                 
        case GL_STACK_UNDERFLOW:
            return "GL_STACK_UNDERFLOW";        
        default:
            return "unknown error code";
    }
}


template<typename T>
static int ogl_check_val(T input, T reference, const char* message)
{
    if(input==reference)
    {
        return true;
    }
    else
    {
	printf("OpenGL Error: %s Error code: %s\n", message, ogl_get_error_code_str(input));
        return false;
    }
}

Texture create2DTexture(int w, int h, TexFormat fmt, const uint8_t* texdata);
Texture createDynamicTexture(int w, int h, TexFormat fmt);
Texture create3DTextureF(int w, int h, int depth);

// texture has to be binded before calling this
void setSamplerParams(TexType tt, TexAddressMode address_mode, TexFilterMode filter);

// pdata_format - specifies format of data provided in pdata
void updateTexture(const Texture& t, void* pdata, TexFormat pdata_format = TF_COUNT);
void destroyTexture(Texture* tex);
// fmt - desired format of returned data
void getTextureData(const Texture& t, int lod, unsigned char* poutdata, TexFormat fmt = TF_COUNT);
unsigned int getPixelSize(const TexFormat fmt);
Texture createPBO(int w, int h, GLenum fmt, int el_size);
void draw_quad(float x0, float y0, float x1, float y1);

struct glsl_program;
void applyTexture(glsl_program* program, int unit, const char* name, GLuint texid);
void applyPBO(glsl_program* program, int unit, const char* name, const Texture pbo, const Texture tex);

void normalize(float (&v)[3]);
void cross(float v1[3], float v2[3], float result[3]);
float dot(float (&v1)[3], float (&v2)[3]);

int glu_InvertMatrixf(const float m[16], float invOut[16]);
void glu_MakeIdentityf(GLfloat m[16]);
void glu_LookAt2(GLdouble eyex, GLdouble eyey, GLdouble eyez, GLdouble centerx,
          GLdouble centery, GLdouble centerz, GLdouble upx, GLdouble upy,
          GLdouble upz);


typedef void (*render_func_t)(int w, int h, void* puserdata);

void draw_in_2d(int w, int h, render_func_t prenderfunc, void* puserdata);

GLuint makeBuffer(GLenum target, const GLvoid* buffer_data, GLsizei buffer_size, GLenum type = GL_STATIC_DRAW);
void updateBuffer(GLuint buf, GLenum target, const GLvoid* buffer_data, GLsizei buffer_size);
void updateBuffer(GLuint buf, GLenum target, const GLvoid* buffer_data, GLsizei buffer_size, GLenum type);

// TODO: rewrite with get_pos, get_norm, get_texcoord handlers
// VertexType should conform to rules: it should have vec3, vec2, vec3 as first members
template<typename VertexType>
void gen_cube_vb(VertexType* vb, size_t count)
{
    assert(vb);
    assert(count >= 36);

    // front faces

	// face v0-v1-v2
    vb[0] = { vec3(1,1,1), vec2(1,1), vec3(0,0,1) };
    vb[1] = { vec3(-1,1,1), vec2(0,1), vec3(0,0,1) };
    vb[2] = { vec3(-1,-1,1), vec2(0,0), vec3(0,0,1) };
	// face v2-v3-v0
    vb[3] = { vec3(-1,-1,1), vec2(0,0), vec3(0,0,1)};
    vb[4] = { vec3(1,-1,1), vec2(1,0), vec3(0,0,1) };
    vb[5] = { vec3(1,1,1), vec2(1,1), vec3(0,0,1) };

	// right faces
    
	// face v0-v3-v4
    vb[6] = { vec3(1,1,1), vec2(0,1), vec3(1,0,0) };
    vb[7] = { vec3(1,-1,1), vec2(0,0), vec3(1, 0,0) };
    vb[8] = { vec3(1,-1,-1), vec2(1,0), vec3(1,0,0) };
	// face v4-v5-v0
    vb[9] = { vec3(1,-1,-1), vec2(1,0), vec3(1,0,0) };
    vb[10] = { vec3(1,1,-1), vec2(1,1), vec3(1, 0,0) };
    vb[11] = { vec3(1,1,1), vec2(0,1), vec3(1,0,0) };

	// top faces
    
	// face v0-v5-v6
    vb[12] = { vec3(1,1,1), vec2(1,0), vec3(0,1,0) };
    vb[13] = { vec3(1,1,-1), vec2(1,1), vec3(0, 1,0) };
    vb[14] = { vec3(-1,1,-1), vec2(0,1), vec3(0,1,0) };
	// face v6-v1-v0
    vb[15] = { vec3(-1,1,-1), vec2(0,1), vec3(0,1,0) };
    vb[16] = { vec3(-1,1,1), vec2(0,0), vec3(0, 1,0) };
    vb[17] = { vec3(1,1,1), vec2(1,0), vec3(0,1,0) };

	// left faces
   
	// face  v1-v6-v7
    vb[18] = { vec3(-1,1,1), vec2(1,1), vec3(-1,0,0) };
    vb[19] = { vec3(-1,1,-1), vec2(0,1), vec3(-1, 0,0) };
    vb[20] = { vec3(-1,-1,-1), vec2(0,0), vec3(-1,0,0) };
	// face v7-v2-v1
    vb[21] = { vec3(-1,-1,-1), vec2(0,0), vec3(-1,0,0) };
    vb[22] = { vec3(-1,-1,1), vec2(1,0), vec3(-1, 0,0) };
    vb[23] = { vec3(-1,1,1), vec2(1,1), vec3(-1,0,0) };

	// bottom faces
   
	// face v7-v4-v3
    vb[24] = { vec3(-1,-1,-1), vec2(0,0), vec3(0,-1,0) };
    vb[25] = { vec3(1,-1,-1), vec2(1,0), vec3(0,-1, 0) };
    vb[26] = { vec3(1,-1,1), vec2(1,1), vec3(0,-1,0) };
	// face v3-v2-v7
    vb[27] = { vec3(1,-1,1), vec2(1,1), vec3(0,-1,0) };
    vb[28] = { vec3(-1,-1,1), vec2(0,1), vec3(0,-1, 0) };
    vb[29] = { vec3(-1,-1,-1), vec2(0,0), vec3(0,-1,0) };

	// back faces
    
	// face v4-v7-v6
    vb[30] = { vec3(1,-1,-1), vec2(0,0), vec3(0,0,-1) };
    vb[31] = { vec3(-1,-1,-1), vec2(1,0), vec3(0,0,-1) };
    vb[32] = { vec3(-1,1,-1), vec2(1,1), vec3(0,0,-1) };
	// face v6-v5-v4
    vb[33] = { vec3(-1,1,-1), vec2(1,1), vec3(0,0,-1) };
    vb[34] = { vec3(1,1,-1), vec2(0,1), vec3(0,0,-1) };
    vb[35] = { vec3(1,-1,-1), vec2(0,0), vec3(0,0,-1) };
}

// BUFFERS ETC.
template <typename VERTEX> 
struct glMesh {
	GLuint	    vb_;
	GLuint	    ib_;
	GLuint		instance_vb_;
	VERTEX*	    pvertices_;
	GLsizei     num_vertices_;
	int*	    pindices_;
	int	    num_indices_;

	uint8_t*	pinstance_data_;
	int		instance_count_;
	int		instance_stride_;

	GLenum	    prim_type_;

				    glMesh();
				    ~glMesh();

	static glMesh<VERTEX>*	    makeMesh(int num_vertices, int num_indices, GLenum prim_type, int instance_array_size = 0, int instance_stride = 0);
	static void				    destroyMesh(glMesh<VERTEX>* pmesh);

	int			    gen_hw(GLenum type = GL_STREAM_DRAW);
	void			    update_hw(GLvoid* vertex_data, GLsizeiptr vsize, GLvoid* index_data, GLsizeiptr isize, GLvoid* instance_data = 0 , GLsizeiptr inst_size = 0);
	void			    update_hw(GLsizeiptr num_cvertices, GLsizeiptr num_indices, GLsizeiptr num_instances);

	typedef glMesh<VERTEX>	    myType;

};

template <typename VERTEX>
glMesh<VERTEX>::glMesh():
vb_(0),ib_(0),instance_vb_(0), pvertices_(0),num_vertices_(0), pindices_(0), num_indices_(0), 
pinstance_data_(0), instance_count_(0), instance_stride_(0), prim_type_(0)
{
}

template <typename VERTEX>
glMesh<VERTEX>* glMesh<VERTEX>::makeMesh(int num_vertices, int num_indices, GLenum prim_type, int instance_count/* = 0*/, int instance_stride/* = 0*/)
{
	glMesh* mesh = new glMesh();
	mesh->num_vertices_ = num_vertices;
	mesh->num_indices_ = num_indices;
	if(num_vertices>0)
	    mesh->pvertices_ = new VERTEX[mesh->num_vertices_];
	if(num_indices>0)
	    mesh->pindices_ = new int[mesh->num_indices_];
	mesh->prim_type_ = prim_type;

	assert(instance_count >= 0);
	if(instance_count > 0)
	{
		assert(instance_stride!=0);

		mesh->instance_count_ = instance_count;
		mesh->instance_stride_ = instance_stride;
		mesh->pinstance_data_ = new unsigned char [mesh->instance_count_ * mesh->instance_stride_];
	}

	return mesh;
}

template <typename VERTEX>
void glMesh<VERTEX>::destroyMesh(glMesh<VERTEX>* pmesh)
{
	assert(pmesh);
	delete pmesh;
}

template <typename VERTEX>
int glMesh<VERTEX>::gen_hw(GLenum type /*= GL_STATIC_DRAW*/)
{
	if(this->pvertices_)
	{
		if(this->vb_ > 0)
			glDeleteBuffers(1, &this->vb_);
		this->vb_ = makeBuffer(GL_ARRAY_BUFFER, this->pvertices_, sizeof(VERTEX)*this->num_vertices_, type);
	}
	if(this->pindices_)
	{
		if(this->ib_ > 0)
			glDeleteBuffers(1, &this->ib_);
		this->ib_ = makeBuffer(GL_ELEMENT_ARRAY_BUFFER, this->pindices_, sizeof(int)*this->num_indices_, type);
	}

	if(this->pinstance_data_)
	{
		if(this->instance_vb_ > 0)
			glDeleteBuffers(1, &this->instance_vb_);
		this->instance_vb_ = makeBuffer(GL_ARRAY_BUFFER, this->pinstance_data_, this->instance_count_ * this->instance_stride_, type);
	}

	return 1;
};

// strange function, ..heh, ..why do I need this?
template <typename VERTEX>
void glMesh<VERTEX>::update_hw(GLvoid* vertex_data, GLsizeiptr vsize, GLvoid* index_data, GLsizeiptr isize, GLvoid* instance_data/* = 0*/, GLsizeiptr inst_count/* = 0*/)
{
	if(this->vb_!=0 && vertex_data && vsize>0)
		updateBuffer(this->vb_, GL_ARRAY_BUFFER, vertex_data, vsize, GL_DYNAMIC_DRAW);

	if(this->ib_!=0 && index_data && isize>0)
		updateBuffer(this->ib_, GL_ELEMENT_ARRAY_BUFFER, index_data, isize, GL_DYNAMIC_DRAW);

	if(this->instance_vb_!=0 && instance_data && isize>0)
	{
		assert(inst_count >= 0);
		updateBuffer(this->instance_vb_, GL_ARRAY_BUFFER, instance_data, inst_count * this->instance_stride_, GL_DYNAMIC_DRAW);
	}
}

template <typename VERTEX>
void glMesh<VERTEX>::update_hw(GLsizeiptr num_vertices, GLsizeiptr num_indices, GLsizeiptr num_instances)
{
	if(this->vb_!=0 && num_vertices>=0)
	{
		GLsizeiptr vsize = (num_vertices!=0 ? num_vertices : num_vertices_) * sizeof(VERTEX); 
		updateBuffer(this->vb_, GL_ARRAY_BUFFER, pvertices_, vsize, GL_DYNAMIC_DRAW);
	}
	if(this->ib_!=0 && num_indices>=0)
	{
		GLsizeiptr isize = (num_indices!=0 ?  num_vertices : num_indices_) * sizeof(int);
		updateBuffer(this->ib_, GL_ELEMENT_ARRAY_BUFFER, pindices_, isize, GL_DYNAMIC_DRAW);
	}
	if(this->instance_vb_!=0 && num_instances>=0)
	{
		GLsizeiptr instsize = (num_instances!=0 ?  num_instances : instance_count_) * instance_stride_;
		updateBuffer(this->instance_vb_, GL_ARRAY_BUFFER, pinstance_data_, instsize, GL_DYNAMIC_DRAW);
	}
}

template <typename VERTEX>
glMesh<VERTEX>::~glMesh()
{
	delete[] pvertices_;
	delete[] pindices_;
	delete[] pinstance_data_;
    	if(this->vb_ > 0)
		glDeleteBuffers(1, &this->vb_);
	if(this->ib_ > 0)
		glDeleteBuffers(1, &this->ib_);
}

void draw_textured_cube(GLuint textureId);

Texture load_texture_from_file(const char* texName);

struct SIMPLE_VERTEX_PTN;
glMesh<SIMPLE_VERTEX_PTN>* make_mesh_from_file(const char* filepath);

template<typename T>
void  draw_mesh_indexed_pt(bool b_wireframe, camera* pcam, glsl_program* pmat, glMesh<T>* pmesh, int num_indices=0)
{
	GLint posaddr = pmat->getAttribLocation("pos");
	assert(posaddr!=-1);
	GLint tcaddr = pmat->getAttribLocation("texcoord");
	assert(tcaddr!=-1);

	if(pcam)
	{
		mat4 proj, view;
		pcam->get_projection(&proj);
		pcam->get_view(&view);
		mat4 viewproj = proj*view;
		pmat->setMat4("ModelViewProjectionMatrix", (const float*)viewproj);
		pmat->apply();
	}
	
	glBindBuffer(GL_ARRAY_BUFFER, pmesh->vb_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmesh->ib_);

	glEnableVertexAttribArray(posaddr);
	glEnableVertexAttribArray(tcaddr);

	glVertexAttribPointer(posaddr, 3, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
	glVertexAttribPointer(tcaddr, 2, GL_FLOAT, GL_FALSE, sizeof(T), BUFFER_OFFSET(3*sizeof(float)));

	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	int idx2draw = num_indices!=0 ? num_indices : pmesh->num_indices_;
	glDrawElements(pmesh->prim_type_, idx2draw , GL_UNSIGNED_INT, 0);
	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glDisableVertexAttribArray(posaddr);
	glDisableVertexAttribArray(tcaddr);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);
}


template<typename T>
void  draw_mesh_indexed_pn(bool b_wireframe, camera* pcam, glsl_program* pmat, glMesh<T>* pmesh)
{
	GLint posaddr = pmat->getAttribLocation("pos");
	assert(posaddr!=-1);
	GLint normaddr = pmat->getAttribLocation("norm");
	assert(normaddr!=-1);

	mat4 proj, view;
	pcam->get_projection(&proj);
	pcam->get_view(&view);
	mat4 viewproj = proj*view;

	pmat->setMat4("ModelViewProjectionMatrix", (const float*)viewproj);
	pmat->apply();
	
	glBindBuffer(GL_ARRAY_BUFFER, pmesh->vb_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pmesh->ib_);

	glEnableVertexAttribArray(posaddr);
	glEnableVertexAttribArray(normaddr);

	glVertexAttribPointer(posaddr, 3, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
	glVertexAttribPointer(normaddr, 3, GL_FLOAT, GL_FALSE, sizeof(T), BUFFER_OFFSET(3*sizeof(float)));

	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(pmesh->prim_type_, pmesh->num_indices_, GL_UNSIGNED_INT, 0);
	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glDisableVertexAttribArray(posaddr);
	glDisableVertexAttribArray(normaddr);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);
}

template<typename T>
void  draw_mesh_ptn(bool b_wireframe, camera* pcam, glsl_program* pmat, glMesh<T>* pmesh)
{
	GLint posaddr = pmat->getAttribLocation("pos");
	assert(posaddr!=-1);
	GLint normaddr = pmat->getAttribLocation("norm");
	//assert(normaddr!=-1);
	GLint tcaddr = pmat->getAttribLocation("texcoord");
	//assert(tcaddr!=-1);
	GLint instaddr = pmat->getAttribLocation("inst_data");

	mat4 proj, view;
	pcam->get_projection(&proj);
	pcam->get_view(&view);
	mat4 viewproj = proj*view;

	pmat->setMat4("ModelViewProjectionMatrix", (const float*)viewproj);
	pmat->apply();
	
	glBindBuffer(GL_ARRAY_BUFFER, pmesh->vb_);

	glEnableVertexAttribArray(posaddr);
	glVertexAttribPointer(posaddr, 3, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
			
	if(tcaddr) {
		glEnableVertexAttribArray(tcaddr);
		glVertexAttribPointer(tcaddr, 2, GL_FLOAT, GL_FALSE, sizeof(T), BUFFER_OFFSET(3*sizeof(float)));
	}

	if(normaddr!=-1) {
		glEnableVertexAttribArray(normaddr);
		glVertexAttribPointer(normaddr, 3, GL_FLOAT, GL_FALSE, sizeof(T), BUFFER_OFFSET(5*sizeof(float)));
	}

	bool draw_instanced = false;
	if(instaddr!=-1 && pmesh->instance_vb_!=0)
	{
		draw_instanced = true;
		glBindBuffer(GL_ARRAY_BUFFER, pmesh->instance_vb_);
		glEnableVertexAttribArray(instaddr);
		glVertexAttribPointer(instaddr, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)0);
		glVertexAttribDivisor(instaddr, 1);
	}
	
	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	if(!draw_instanced)
		glDrawArrays(pmesh->prim_type_, 0, pmesh->num_vertices_);
	else
		glDrawArraysInstanced(pmesh->prim_type_, 0, pmesh->num_vertices_, pmesh->instance_count_);
	if(b_wireframe)
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glDisableVertexAttribArray(posaddr);
	glDisableVertexAttribArray(normaddr);
	glDisableVertexAttribArray(tcaddr);

	if(draw_instanced)
	{
		glDisableVertexAttribArray(instaddr);
		glVertexAttribDivisor(instaddr, 0);
	}


	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);
}

template<typename MeshBuffer>
void subdivide(MeshBuffer& mb, int& vb_offset, int& ib_offset, vec3 a, vec3 b, vec3 c, int subdiv_count)
{
    vec3 ab = normalize(lerp(a, b, 0.5f));
    vec3 bc = normalize(lerp(b, c, 0.5f));
    vec3 ca = normalize(lerp(c, a, 0.5f));
    static const bool ccw = true;

    if(0 == subdiv_count-1) { 
        vec3 n = normalize(cross(ca - a, ab - a));
        mb.p(vb_offset, a); mb.p(vb_offset+1, ccw?ca:ab); mb.p(vb_offset+2, ccw?ab:ca);
        mb.n(vb_offset, n); mb.n(vb_offset+1, n); mb.n(vb_offset+2, n);
        mb.i(ib_offset, ib_offset); mb.i(ib_offset+1, ib_offset+1); mb.i(ib_offset+2, ib_offset+2);
        vb_offset+=3;
        ib_offset+=3;

        n = normalize(cross(bc - ca, ab - ca));
        mb.p(vb_offset, ca); mb.p(vb_offset+1, ccw?bc:ab); mb.p(vb_offset+2, ccw?ab:bc); 
        mb.n(vb_offset, n); mb.n(vb_offset+1, n); mb.n(vb_offset+2, n);
        mb.i(ib_offset, ib_offset); mb.i(ib_offset+1, ib_offset+1); mb.i(ib_offset+2, ib_offset+2);
        vb_offset+=3;
        ib_offset+=3;

        n = normalize(cross(c - ca, bc - ca));
        mb.p(vb_offset, ca); mb.p(vb_offset+1, ccw?c:bc); mb.p(vb_offset+2, ccw?bc:c); 
        mb.n(vb_offset, n); mb.n(vb_offset+1, n); mb.n(vb_offset+2, n);
        mb.i(ib_offset, ib_offset); mb.i(ib_offset+1, ib_offset+1); mb.i(ib_offset+2, ib_offset+2);
        vb_offset+=3;
        ib_offset+=3;

        n = normalize(cross(bc - ab, b - ab));
        mb.p(vb_offset, ab); mb.p(vb_offset+1, ccw?bc:b); mb.p(vb_offset+2, ccw?b:bc);
        mb.n(vb_offset, n); mb.n(vb_offset+1, n); mb.n(vb_offset+2, n);
        mb.i(ib_offset, ib_offset); mb.i(ib_offset+1, ib_offset+1); mb.i(ib_offset+2, ib_offset+2);
        vb_offset+=3;
        ib_offset+=3;
    } else {
        subdivide(mb, vb_offset, ib_offset, a, ab, ca, subdiv_count-1);
        subdivide(mb, vb_offset, ib_offset, ca, ab, bc, subdiv_count-1);
        subdivide(mb, vb_offset, ib_offset, ca, bc, c, subdiv_count-1);
        subdivide(mb, vb_offset, ib_offset, ab, b, bc, subdiv_count-1);
    }
}

template<typename MeshBuffer>
void generate_tetrahedron(MeshBuffer& mb)
{
    const int num_vertices = 4;
    const int num_indices = 4 * 3;
    const float oo_sqrt2 = 1.0f / sqrt(2.0f);
    const vec3 A(0.0f, 1.0f, oo_sqrt2);
    const vec3 B(0.0f, -1.0f, oo_sqrt2);
    const vec3 C(1.0f, 0.0f, -oo_sqrt2);
    const vec3 D(-1.0f, 0.0f, -oo_sqrt2);

    mb.allocate_vb(num_vertices);
    mb.allocate_ib(num_indices);

    vec3 a = A;
    vec3 b = B;
    vec3 c = C;
    vec3 d = D;

    vec3 na = normalize(vec3(a.x, a.y, a.z));
    vec3 nb = normalize(vec3(b.x, b.y, b.z));
    vec3 nc = normalize(vec3(c.x, c.y, c.z));
    vec3 nd = normalize(vec3(d.x, d.y, d.z));

    // smoothed normals
    mb.p(0, na);
    mb.n(0, na);
    mb.p(1, nb);
    mb.n(1, nb);
    mb.p(2, nc);
    mb.n(2, nc);
    mb.p(3, nd);
    mb.n(3, nd);

    int i=0;
    mb.i(i++, 1);
    mb.i(i++, 0);
    mb.i(i++, 2);

    mb.i(i++,1);
    mb.i(i++,2);
    mb.i(i++,3);

    mb.i(i++,1);
    mb.i(i++,3);
    mb.i(i++,0);

    mb.i(i++,0);
    mb.i(i++,3);
    mb.i(i++,2);
}

template<typename MeshBuffer>
void generate_sphere(MeshBuffer& mb, unsigned int subdiv_count)
{
    const float oo_sqrt2 = 1.0f / sqrt(2.0f);
    const vec3 A(0.0f, 1.0f, oo_sqrt2);
    const vec3 B(0.0f, -1.0f, oo_sqrt2);
    const vec3 C(1.0f, 0.0f, -oo_sqrt2);
    const vec3 D(-1.0f, 0.0f, -oo_sqrt2);

	const unsigned int num_tris = (unsigned int)pow(4, subdiv_count + 1);
	const unsigned int num_vert = 3 * num_tris;
    mb.allocate_vb(num_vert);
    mb.allocate_ib(num_vert);

    vec3 a = A;
    vec3 b = B;
    vec3 c = C;
    vec3 d = D;

    vec3 na = normalize(vec3(a.x, a.y, a.z));
    vec3 nb = normalize(vec3(b.x, b.y, b.z));
    vec3 nc = normalize(vec3(c.x, c.y, c.z));
    vec3 nd = normalize(vec3(d.x, d.y, d.z));

    int ib_offset = 0;
    int vb_offset = 0;
    subdivide(mb, vb_offset, ib_offset, nb, na, nc, subdiv_count);
    subdivide(mb, vb_offset, ib_offset, nb, nc, nd, subdiv_count);
    subdivide(mb, vb_offset, ib_offset, nb, nd, na, subdiv_count);
    subdivide(mb, vb_offset, ib_offset, na, nd, nc, subdiv_count);
    assert(vb_offset<=(int)mb.vb_size_);
    assert(ib_offset<=(int)mb.ib_size_);
}

template<typename MeshBuffer>
void generate_cube(MeshBuffer& mb, const vec3 scale, const vec3 offset)
{
    mb.allocate_vb(36);
    vec3 xyz = vec3(1,1,1)*scale + offset;
    vec3 _yz = vec3(-1,1,1)*scale + offset;
    vec3 x_z = vec3(1,-1,1)*scale + offset;
    vec3 xy_ = vec3(1,1,-1)*scale + offset;
    vec3 x__ = vec3(1,-1,-1)*scale + offset;
    vec3 _y_ = vec3(-1,1,-1)*scale + offset;
    vec3 __z = vec3(-1,-1,1)*scale + offset;
    vec3 ___ = vec3(-1,-1,-1)*scale + offset;

    vec3 nx = vec3(1,0,0);
    vec3 ny = vec3(0,1,0);
    vec3 nz = vec3(0,0,1);

    vec2 uv = vec2(1,1);
    vec2 u_ = vec2(1,0);
    vec2 _v = vec2(0,1);
    vec2 __ = vec2(0,0);

    // front faces

	// face v0-v1-v2
    mb.p(0, xyz); mb.uv(0, uv); mb.n(0, nz);
    mb.p(1, _yz); mb.uv(1, _v); mb.n(1, nz);
    mb.p(2, __z); mb.uv(2, __); mb.n(2, nz);
	// face v2-v3-v0
    mb.p(3, __z); mb.uv(3, __); mb.n(3, nz);
    mb.p(4, x_z); mb.uv(4, u_); mb.n(4, nz);
    mb.p(5, xyz), mb.uv(5, uv); mb.n(5, nz);

	// right faces
    
	// face v0-v3-v4
    mb.p(6, xyz), mb.uv(6, _v), mb.n(6, nx);
    mb.p(7, x_z), mb.uv(7, __), mb.n(7, nx);
    mb.p(8, x__), mb.uv(8, u_), mb.n(8, nx);
	// face v4-v5-v0
    mb.p(9, x__), mb.uv(9, u_), mb.n(9, nx);
    mb.p(10, xy_), mb.uv(10, uv), mb.n(10, nx);
    mb.p(11, xyz), mb.uv(11, _v), mb.n(11, nx);

	// top faces
    
	// face v0-v5-v6
    mb.p(12, xyz), mb.uv(12, u_), mb.n(12, ny);
    mb.p(13, xy_), mb.uv(13, uv), mb.n(13, ny);
    mb.p(14, _y_), mb.uv(14, _v), mb.n(14, ny);
	// face v6-v1-v0
    mb.p(15, _y_), mb.uv(15, _v), mb.n(15, ny);
    mb.p(16, _yz), mb.uv(16, __), mb.n(16, ny);
    mb.p(17, xyz), mb.uv(17, u_), mb.n(17, ny);

	// left faces
   
	// face  v1-v6-v7
    mb.p(18, _yz), mb.uv(18, uv), mb.n(18, -nx);
    mb.p(19, _y_), mb.uv(19, _v), mb.n(19, -nx);
    mb.p(20, ___), mb.uv(20, __), mb.n(20, -nx);
	// face v7-v2-v1
    mb.p(21, ___), mb.uv(21, __), mb.n(21, -nx);
    mb.p(22, __z), mb.uv(22, u_), mb.n(22, -nx);
    mb.p(23, _yz), mb.uv(23, uv), mb.n(23, -nx);

	// bottom faces
   
	// face v7-v4-v3
    mb.p(24, ___), mb.uv(24, __), mb.n(24, -ny);
    mb.p(25, x__), mb.uv(25, u_), mb.n(25, -ny);
    mb.p(26, x_z), mb.uv(26, uv), mb.n(26, -ny);
	// face v3-v2-v7
    mb.p(27, x_z), mb.uv(27, uv), mb.n(27, -ny);
    mb.p(28, __z), mb.uv(28, _v), mb.n(28, -ny);
    mb.p(29, ___), mb.uv(29, __), mb.n(29, -ny);

	// back faces
    
	// face v4-v7-v6
    mb.p(30, x__), mb.uv(30, __), mb.n(30, -nz);
    mb.p(31, ___), mb.uv(31, u_), mb.n(31, -nz);
    mb.p(32, _y_), mb.uv(32, uv), mb.n(32, -nz);
	// face v6-v5-v4
    mb.p(33, _y_), mb.uv(33, uv), mb.n(33, -nz);
    mb.p(34, xy_), mb.uv(34, _v), mb.n(34, -nz);
    mb.p(35, x__), mb.uv(35, __), mb.n(35, -nz);
}

template <typename MeshBuffer>
void generate_axes(MeshBuffer& mb)
{
    generate_cube(mb, vec3(1.0f,0.1f,0.1f), vec3(1.0f, 0.0f, 0.0f));
    mb.set_offset(mb.vb_size_);
    generate_cube(mb, vec3(0.1f,1.0f,0.1f), vec3(0.0f, 1.0f, 0.0f));
    mb.set_offset(mb.vb_size_);
    generate_cube(mb, vec3(0.1f,0.1f,1.0f), vec3(0.0f, 0.0f, 1.0f));
    mb.set_offset(mb.vb_size_);
    generate_cube(mb, vec3(0.15f), vec3(0.0f));
}

// generates torus in XY plane
template <typename MeshBuffer>
void generate_torus_no_uv(MeshBuffer &mb, float radius, float thickness,
					unsigned int toroidal_sections, unsigned int poloidal_sections) {
	gosASSERT(radius != 0 && thickness != 0 && toroidal_sections > 2 && poloidal_sections > 2);

	const unsigned int num_tris = toroidal_sections * poloidal_sections * 2;
	const unsigned int num_vert = (toroidal_sections + 1)* (poloidal_sections + 1);
	mb.allocate_vb(num_vert);
	mb.allocate_ib(num_tris * 3);

	unsigned vb_offset = 0;
	unsigned int section_index_start = 0;
	for (unsigned int ts = 0; ts < toroidal_sections; ++ts) {
		float phi = (ts/(float)toroidal_sections) * 2.0f * M_PI;
		vec3 sec_center = vec3(radius * cos(phi), radius * sin(phi), 0.0f);
		for (unsigned int ps = 0; ps < poloidal_sections; ++ps) {
			float theta = (ps / (float)poloidal_sections) * 2.0f * M_PI;
			vec3 p = sec_center + vec3(thickness * cos(theta) * cos(phi),
									   thickness * cos(theta) * sin(phi), thickness * sin(theta));
			mb.p(vb_offset, p);
			mb.n(vb_offset, normalize(p - sec_center));
			vb_offset++;
		}
	}

    assert(vb_offset == (unsigned int)mb.vb_size_);

	unsigned int idx = 0;
	for (unsigned int ts = 0; ts < toroidal_sections; ++ts) {
		unsigned int sec0_start = ts * poloidal_sections;
		unsigned int sec1_start = ((ts + 1) % toroidal_sections) * poloidal_sections;
		for (unsigned int ps = 0; ps < poloidal_sections; ++ps) {

			unsigned int v0 = sec0_start + ps;
			unsigned int v1 = sec0_start + (ps + 1) % poloidal_sections;
			unsigned int v2 = sec1_start + ps;
			unsigned int v3 = sec1_start + (ps + 1) % poloidal_sections;

			// v1 +--------+ v3
			// v0 +--------+ v2

			mb.i(idx++, v0);
			mb.i(idx++, v2);
			mb.i(idx++, v1);

			mb.i(idx++, v2);
			mb.i(idx++, v3);
			mb.i(idx++, v1);
		}
	}

    assert(idx == (unsigned int)mb.ib_size_);
}

// generates torus in XY plane
template <typename MeshBuffer>
void generate_torus(MeshBuffer &mb, float radius, float thickness,
					unsigned int toroidal_sections, unsigned int poloidal_sections) {
	gosASSERT(radius != 0 && thickness != 0 && toroidal_sections > 2 && poloidal_sections > 2);

	const unsigned int num_tris = toroidal_sections * poloidal_sections * 2;
	// we add one vert which will have same position as 0 but different uv
	// for correct uv handling (if we do not need uv we could just wrap around as in no_uv version
	const unsigned int num_vert = (toroidal_sections + 1)* (poloidal_sections + 1);
	mb.allocate_vb(num_vert);
	mb.allocate_ib(num_tris * 3);

	unsigned vb_offset = 0;
	unsigned int section_index_start = 0;
	for (unsigned int ts = 0; ts <= toroidal_sections; ++ts) {
		float phi = (ts/(float)toroidal_sections) * 2.0f * M_PI;
		vec3 sec_center = vec3(radius * cos(phi), radius * sin(phi), 0.0f);
		for (unsigned int ps = 0; ps <= poloidal_sections; ++ps) {
			float theta = (ps / (float)poloidal_sections) * 2.0f * M_PI;
			vec3 p = sec_center + vec3(thickness * cos(theta) * cos(phi),
									   thickness * cos(theta) * sin(phi), thickness * sin(theta));
			mb.p(vb_offset, p);
			mb.n(vb_offset, normalize(p - sec_center));
			mb.uv(vb_offset, vec2(ts/(float)toroidal_sections, ps/(float)poloidal_sections));
			vb_offset++;
		}
	}

    assert(vb_offset == (unsigned int)mb.vb_size_);

	unsigned int idx = 0;
	for (unsigned int ts = 0; ts < toroidal_sections; ++ts) {
		unsigned int sec0_start = ts * (poloidal_sections + 1);
		unsigned int sec1_start = (ts + 1) * (poloidal_sections + 1);
		for (unsigned int ps = 0; ps < poloidal_sections; ++ps) {

			unsigned int v0 = sec0_start + ps;
			unsigned int v1 = sec0_start + (ps + 1);
			unsigned int v2 = sec1_start + ps;
			unsigned int v3 = sec1_start + (ps + 1);

			// v1 +--------+ v3
			// v0 +--------+ v2

			mb.i(idx++, v0);
			mb.i(idx++, v2);
			mb.i(idx++, v1);

			mb.i(idx++, v2);
			mb.i(idx++, v3);
			mb.i(idx++, v1);
		}
	}

    assert(idx == (unsigned int)mb.ib_size_);
}

#endif // __GL_UTILS_H__
