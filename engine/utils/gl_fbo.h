#ifndef __GL_FBO_H__
#define __GL_FBO_H__

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif
#include <GL/gl.h>
#include "gl_render_constants.h"

GLuint createRenderTexture(int w, int h, int fmt = GL_RGB8, int int_fmt = GL_RGB, int type = GL_UNSIGNED_BYTE, GLint min_filter = GL_NEAREST, GLint mag_filter = GL_NEAREST);
GLuint createRenderBuffer(int w, int h, int format = -1);
GLuint createRenderBuffer(int w, int h, const  TexFormat format);

void setRenderTexture(int tex, int index = 0);
void setRenderBuffer(int buffer, int index = -1);

GLuint createFrameBuffer();
void setFrameBuffer(GLuint fb);

// check FBO completeness
bool checkFramebufferStatus();

void draw_quad(GLuint texture, float aspect);
void draw_quad(float aspect);

#endif // __GL_FBO_H__
