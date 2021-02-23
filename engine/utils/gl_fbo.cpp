#ifdef PLATFORM_WINDOWS
#define NOMINMAX
#include <windows.h>
#endif
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <cassert>

#include "utils/gl_fbo.h"

GLuint createRenderTexture(uint32_t w, uint32_t h, int int_fmt, uint32_t levels)
{
	GLuint rt;
	glGenTextures(1, &rt);

	glBindTexture(GL_TEXTURE_2D, rt);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexStorage2D(GL_TEXTURE_2D, levels, int_fmt, w, h);

	glBindTexture(GL_TEXTURE_2D, 0);

	return rt;
}

GLuint createRenderBuffer(int w, int h, const TexFormat format)
{
    return createRenderBuffer(w, h, getInternalTextureFormat(format));
}

GLuint createRenderBuffer(int w, int h, int format)
{
	GLuint depth_rb;
	glGenRenderbuffers(1, &depth_rb);
	
	glBindRenderbuffer(GL_RENDERBUFFER, depth_rb);
	
	int fmt = GL_DEPTH_COMPONENT24;
	if(format!=-1)
		fmt = format;

	glRenderbufferStorage(GL_RENDERBUFFER, fmt, w, h);

    glBindRenderbuffer(GL_RENDERBUFFER, 0);

	return depth_rb;
}

void setRenderTexture(int tex, int index)
{
	int attach = GL_COLOR_ATTACHMENT0;

	if(index==-1)
		attach = GL_DEPTH_ATTACHMENT;
	else
		attach = GL_COLOR_ATTACHMENT0 + index;
	
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, attach, GL_TEXTURE_2D, tex, 0);
}

void setRenderBuffer(int buffer, int index)
{
	int attach = GL_COLOR_ATTACHMENT0;

	if(index==-1)
		attach = GL_DEPTH_ATTACHMENT;
	else
		attach = GL_COLOR_ATTACHMENT0 + index;

	glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, attach, GL_RENDERBUFFER, buffer);
}


GLuint createFrameBuffer()
{
	GLuint fb;
	glGenFramebuffers(1, &fb);
	return fb;
}

void setFrameBuffer(GLuint fb)
{
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fb);
}


///////////////////////////////////////////////////////////////////////////////
// check FBO completeness
///////////////////////////////////////////////////////////////////////////////
bool checkFramebufferStatus()
{
    // check FBO status
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status)
    {
    case GL_FRAMEBUFFER_COMPLETE:
        //std::cout << "Framebuffer complete." << std::endl;
        return true;

    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        std::cout << "[ERROR] Framebuffer incomplete: Attachment is NOT complete." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        std::cout << "[ERROR] Framebuffer incomplete: No image is attached to FBO." << std::endl;
        return false;
/*
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
        std::cout << "[ERROR] Framebuffer incomplete: Attached images have different dimensions." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
        std::cout << "[ERROR] Framebuffer incomplete: Color attached images have different internal formats." << std::endl;
        return false;
*/
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        std::cout << "[ERROR] Framebuffer incomplete: Draw buffer." << std::endl;
        return false;

    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        std::cout << "[ERROR] Framebuffer incomplete: Read buffer." << std::endl;
        return false;

    case GL_FRAMEBUFFER_UNSUPPORTED:
        std::cout << "[ERROR] Framebuffer incomplete: Unsupported by FBO implementation." << std::endl;
        return false;

    default:
        std::cout << "[ERROR] Framebuffer incomplete: Unknown error." << std::endl;
        return false;
    }
}

void downlsampleFboDepth(GLuint src_fbo, GLuint dst_fbo, uint32_t src_width,
                         uint32_t src_height, uint32_t dst_width,
                         uint32_t dst_height) {

    glBindFramebuffer(GL_READ_FRAMEBUFFER, src_fbo);//CHECK_GL_ERROR();
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dst_fbo);//CHECK_GL_ERROR();

    glBlitFramebuffer(0, 0, src_width, src_height,
                        0, 0, dst_width, dst_height,
                        GL_DEPTH_BUFFER_BIT, GL_NEAREST);//CHECK_GL_ERROR();
}

void  draw_quad(float aspect)
{
	float r = aspect;

	glBegin(GL_QUADS);
		
		glTexCoord2f(0,0);
		glVertex2f(-1*r,-1);
		
		glTexCoord2f(1,0);
		glVertex2f(1*r, -1);
		
		glTexCoord2f(1,1);
		glVertex2f(1*r, 1);
		
		glTexCoord2f(0,1);
		glVertex2f(-1*r, 1);
	glEnd();
}




void  draw_quad(GLuint texture, float aspect)
{
	if(texture!=0)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture);
	}

	draw_quad(aspect);

	if(texture!=0)
	{
		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);
	}
}


