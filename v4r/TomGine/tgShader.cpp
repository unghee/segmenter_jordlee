/* tgShader
 *
 * Copyright (C) 2005, Maurizio Monge <maurizio.monge@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "tgShader.h"
#include "tgError.h"
#include <string.h>
#include <stdarg.h>
#include <stdexcept>

#ifdef HAVE_GTK
#include <gtk/gtk.h>
#endif

#include <GL/glu.h>

using namespace TomGine;

static const char *gl_type_name(GLenum type)
{
    switch(type)
    {
        case GL_FLOAT:             return "float";
        case GL_FLOAT_VEC2_ARB:    return "vec2";
        case GL_FLOAT_VEC3_ARB:    return "vec3";
        case GL_FLOAT_VEC4_ARB:    return "vec4";
        case GL_FLOAT_MAT2_ARB:    return "mat2";
        case GL_FLOAT_MAT3_ARB:    return "mat3";
        case GL_FLOAT_MAT4_ARB:    return "mat4";
        case GL_INT:               return "int";
        case GL_INT_VEC2_ARB:      return "ivec2";
        case GL_INT_VEC3_ARB:      return "ivec3";
        case GL_INT_VEC4_ARB:      return "ivec4";
        case GL_BOOL_ARB:          return "bool";
        case GL_BOOL_VEC2_ARB:     return "bvec2";
        case GL_BOOL_VEC3_ARB:     return "bvec3";
        case GL_BOOL_VEC4_ARB:     return "bvec4";
        case GL_SAMPLER_1D:        return "sampler1D";
        case GL_SAMPLER_2D:        return "sampler2D";
        case GL_SAMPLER_3D:        return "sampler3D";
        case GL_SAMPLER_CUBE:      return "samplerCube";
        case GL_SAMPLER_1D_SHADOW: return "sampler1DShadow";
        case GL_SAMPLER_2D_SHADOW: return "sampler2DShadow";
        default:
        {
            static char tmp[64];
            snprintf(tmp,64,"?0x%x?", type );
            return tmp;
        }
    }
}

void tgShader::dumpVars()
{
    int nv;
    glGetObjectParameterivARB( program, GL_OBJECT_ACTIVE_UNIFORMS_ARB, &nv);
    printf("UNIFORM variables (%d):\n",nv);

    for(int i=0;i<nv;i++)
    {
        GLenum type;
        int size;
        char vname[4096];
        glGetActiveUniformARB(program,i,4096,NULL,&size,&type,vname);
        printf("  %s %s;\n", gl_type_name(type),vname);
    }

    glGetObjectParameterivARB( program, GL_OBJECT_ACTIVE_ATTRIBUTES_ARB, &nv);
    printf("ATTRIBUTE variables (%d):\n",nv);
    for(int i=0;i<nv;i++)
    {
        GLenum type;
        int size;
        char vname[4096];
        glGetActiveAttribARB(program,i,4096,NULL,&size,&type,vname);
        printf("  %s %s;\n", gl_type_name(type),vname);
    }
}

void tgShader::printInfoLog(GLhandleARB obj, const char *msg, ...)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetObjectParameterivARB(obj, GL_OBJECT_INFO_LOG_LENGTH_ARB,
                              &infologLength);
    if(infologLength > 1)
    {
        va_list va;
        va_start(va, msg);
        infoLog = (char *)malloc(infologLength);
        glGetInfoLogARB(obj, infologLength, &charsWritten, infoLog);
#ifdef HAVE_GTK
        char *m = g_strdup_vprintf(msg, va);
        GtkWidget *dialog = gtk_message_dialog_new(NULL,GTK_DIALOG_MODAL,
                GTK_MESSAGE_ERROR,GTK_BUTTONS_OK,"%s (%d):",m,infologLength);
        g_free(m);
        gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(dialog),"%s",infoLog);
        gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_destroy(dialog);
#else
        vprintf(msg, va);
        printf(" (%d):\n",infologLength);
        printf("%s",infoLog);
#endif
        va_end(va);
        free(infoLog);
    }
}

bool tgShader::getStatus(){
    if(program)
        return true;
    
    return false;
}

tgShader::tgShader(	const char *vertex_file, const char *fragment_file,
					const char *vertex_header_file, const char *fragment_header_file)
{
    GLint status;
    const char *vs = read_text_file(vertex_file);
    const char *fs = read_text_file(fragment_file);
    const char *vhs = read_text_file(vertex_header_file);
    const char *fhs = read_text_file(fragment_header_file);

    char charbuffer[512];
	
	if( vertex_file && !vs ){
		sprintf(charbuffer, "[tgShader::tgShader] Error loading file for vertex shader '%s'.\n"
		    "  Have you installed TomGine (make install)?\n", vertex_file);
		throw std::runtime_error(charbuffer);
	}

	if( fragment_file && !fs ){
		sprintf(charbuffer, "[tgShader::tgShader] Error loading file for fragment shader '%s'.\n"
        "  Have you installed TomGine (make install)?\n", fragment_file);
		throw std::runtime_error(charbuffer);
	}

	if( vertex_header_file && !vhs ){
		sprintf(charbuffer, "[tgShader::tgShader] Error loading header file for vertex shader '%s'.\n"
        "  Have you installed TomGine (make install)?\n", vertex_header_file);
		throw std::runtime_error(charbuffer);
	}

	if( fragment_header_file && !fhs ){
		sprintf(charbuffer, "[tgShader::tgShader] Error loading header file for fragment shader '%s'.\n"
        "  Have you installed TomGine (make install)?\n", fragment_header_file);
		throw std::runtime_error(charbuffer);
	}

	if(vertex_header_file)  {
		char *tmp = (char*)malloc(strlen(vhs)+strlen(vs)+1);
		strcpy(tmp,vhs);
		strcat(tmp,vs);
		free((void*)vs);
		vs = tmp;
    }

	if(fragment_header_file)  {
		char *tmp = (char*)malloc(strlen(fhs)+strlen(fs)+1);
		strcpy(tmp,fhs);
		strcat(tmp,fs);
		free((void*)fs);
		fs = tmp;
    }

    if(vs)
    {
        vertex = glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
        glShaderSourceARB(vertex, 1, &vs, NULL);
        glCompileShaderARB(vertex);

        glGetObjectParameterivARB( vertex, GL_OBJECT_COMPILE_STATUS_ARB, &status);
        if(!status)
        {
            printInfoLog(vertex,"[tgShader::tgShader] Error compiling vertex shader \"%s\"",vertex_file);
            program = 0;
            return;
        }
        free((void*)vs);
    }
    else
        vertex = 0;

    if(fs)
    {
        fragment = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
        glShaderSourceARB(fragment, 1, &fs,NULL);
        glCompileShaderARB(fragment);

        glGetObjectParameterivARB( fragment, GL_OBJECT_COMPILE_STATUS_ARB, &status);
        if(!status)
        {
            printInfoLog(fragment,"[tgShader::tgShader] Error compiling fragment shader \"%s\"",fragment_file);
            program = 0;
            return;
        }
        free((void*)fs);
    }
    else
        fragment = 0;

    if(fragment==0 && vertex==0)
    {
        program = 0;
        return;
    }

    program = glCreateProgramObjectARB();
    if(vertex!=0)
    {
        glAttachObjectARB(program,vertex);
        glDeleteObjectARB(vertex);
    }
    if(fragment!=0)
    {
        glAttachObjectARB(program,fragment);
        glDeleteObjectARB(fragment);
    }
    glLinkProgramARB(program);

    glGetObjectParameterivARB( program, GL_OBJECT_LINK_STATUS_ARB, &status);
    if(!status)
    {
        printInfoLog(program,"Error linking program with \"%s\" and \"%s\"",
                                                    vertex_file,fragment_file);
        glDeleteObjectARB(program);
        program = 0;
        return;
    }

    glValidateProgramARB(program);
    glGetObjectParameterivARB( program, GL_OBJECT_VALIDATE_STATUS_ARB,&status);
    printInfoLog(program,"%s validating program",status?"Information":"Error");
    if(!status)
    {
        glDeleteObjectARB(program);
        program = 0;
    }

    //dumpVars();
#ifdef DEBUG
    tgCheckError("[tgShader::tgShader]");
#endif
}

tgShader::~tgShader()
{
	if(program!=0){
        glDeleteObjectARB(program);
    }
#ifdef DEBUG
    tgCheckError("[tgShader::~tgShader]");
#endif
}

void tgShader::bind()
{
    if(program)
        glUseProgramObjectARB(program);
}

void tgShader::unbind()
{
    if(program)
        glUseProgramObjectARB(0);
}

GLuint tgShader::getAttribLoc(const char *attr)
{
    return glGetAttribLocationARB( program, attr);
}

GLint tgShader::getUniformLoc(const char* var)
{
    return glGetUniformLocationARB(program,var);
}

void tgShader::setUniform(const char* var,int f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1iARB(loc,f);
}

void tgShader::setUniform(const char* var,unsigned f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1iARB(loc,(int)f);
}

void tgShader::setUniform(const char* var,int n,const int* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1ivARB(loc,n,f);
}

void tgShader::setUniform(const char* var,float f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fARB(loc,f);
}

void tgShader::setUniform(const char* var,int n,const float* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,n,f);
    //printf("f: %f %f %f\n", f[0],f[1],f[2]);
}

void tgShader::setUniform(const char* var,vec2 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform2fvARB(loc,1,f.v);
}

void tgShader::setUniform(const char* var,int n,vec2* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,2*n,f->v);
}

void tgShader::setUniform(const char* var,vec3 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform3fvARB(loc,1,f.v);
}

void tgShader::setUniform(const char* var,int n,vec3* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,3*n,f->v);
}

void tgShader::setUniform(const char* var,vec4 f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform4fv(loc,1,f.v);
}

void tgShader::setUniform(const char* var,int n,vec4* f)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniform1fvARB(loc,4*n,f->v);
}

void tgShader::setUniform(const char* var,mat3 f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix3fvARB(loc,1,transpose,f.mat);
}

void tgShader::setUniform(const char* var,int n,mat3* f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix3fvARB(loc,n,transpose,f->mat);
}

void tgShader::setUniform(const char* var,mat4 f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix4fvARB(loc,1,transpose,f.mat);
}

void tgShader::setUniform(const char* var,int n,mat4* f,bool transpose)
{
    int loc = glGetUniformLocationARB(program,var);
    glUniformMatrix4fvARB(loc,n,transpose,f->mat);
}

void* TomGine::malloc_align(size_t size, int align)
{
    char *tmp = (char*)malloc(size+align-1+sizeof(void*));
    char *addr = tmp+align-1+sizeof(void*);
    addr -= (long)addr & (align-1);
    ((void**)addr)[-1] = tmp;
    return addr;
}

void TomGine::free_align(void *addr)
{
    void *tmp = ((void**)addr)[-1];
    free(tmp);
}

char* TomGine::read_text_file(const char* file)
{
    if(!file) return NULL;

    FILE *f = fopen(file ,"r");
    if(!f) return NULL;

    fseek(f, 0, SEEK_END);
    int sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *retv = (char*)malloc(sz+1);
    fread(retv, sz, 1, f);
    retv[sz] = 0;
    fclose(f);

    return retv;
}

