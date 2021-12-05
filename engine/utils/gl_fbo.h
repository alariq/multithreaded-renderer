#ifndef __GL_FBO_H__
#define __GL_FBO_H__

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif
#include "gl_render_constants.h"
#include <GL/gl.h>

GLuint createRenderTexture(uint32_t w, uint32_t h, int int_fmt, uint32_t levels);
GLuint createRenderBuffer(int w, int h, int format = -1);
GLuint createRenderBuffer(int w, int h, const TexFormat format);

void setRenderTexture(int tex, int index = 0);
void setRenderBuffer(int buffer, int index = -1);

GLuint createFrameBuffer();
void setFrameBuffer(GLuint fb);

// check FBO completeness
bool checkFramebufferStatus(GLenum target);
void downlsampleFboDepth(GLuint src_fbo, GLuint dst_fbo, uint32_t src_width,
						 uint32_t src_height, uint32_t dst_width, uint32_t dst_height);

void draw_quad(GLuint texture, float aspect);
void draw_quad(float aspect);

void copyFramebufferToCPU(GLuint framebuffer_id, GLenum mode, TexFormat fmt, int width,
						  int height, void* out_data);

void copyFramebuffer(GLuint src_fbo, GLuint dst_fbo, GLenum mode, uint32_t w, uint32_t h);

#endif // __GL_FBO_H__
