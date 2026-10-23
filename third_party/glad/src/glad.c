#include <glad/glad.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>
static HMODULE glad_libGL = NULL;
static void* glad_get_proc(GLADloadproc userptr, const char* name) {
    void* res = NULL;
    if (userptr != NULL) {
        res = userptr(name);
    }
    if (res == NULL && glad_libGL != NULL) {
        res = (void*)GetProcAddress(glad_libGL, name);
    }
    return res;
}
#else
static void* glad_get_proc(GLADloadproc userptr, const char* name) {
    return userptr ? userptr(name) : NULL;
}
#endif

PFNGLVIEWPORTPROC glad_glViewport = NULL;
PFNGLCLEARCOLORPROC glad_glClearColor = NULL;
PFNGLCLEARPROC glad_glClear = NULL;
PFNGLENABLEPROC glad_glEnable = NULL;
PFNGLDISABLEPROC glad_glDisable = NULL;
PFNGLDEPTHFUNCPROC glad_glDepthFunc = NULL;
PFNGLDEPTHMASKPROC glad_glDepthMask = NULL;
PFNGLBLENDFUNCPROC glad_glBlendFunc = NULL;
PFNGLCULLFACEPROC glad_glCullFace = NULL;
PFNGLFRONTFACEPROC glad_glFrontFace = NULL;
PFNGLLINEWIDTHPROC glad_glLineWidth = NULL;
PFNGLPOLYGONOFFSETPROC glad_glPolygonOffset = NULL;
PFNGLPOLYGONMODEPROC glad_glPolygonMode = NULL;
PFNGLGETSTRINGPROC glad_glGetString = NULL;
PFNGLGETERRORPROC glad_glGetError = NULL;
PFNGLDRAWBUFFERPROC glad_glDrawBuffer = NULL;
PFNGLREADBUFFERPROC glad_glReadBuffer = NULL;
PFNGLREADPIXELSPROC glad_glReadPixels = NULL;

PFNGLGENTEXTURESPROC glad_glGenTextures = NULL;
PFNGLDELETETEXTURESPROC glad_glDeleteTextures = NULL;
PFNGLBINDTEXTUREPROC glad_glBindTexture = NULL;
PFNGLTEXIMAGE2DPROC glad_glTexImage2D = NULL;
PFNGLTEXPARAMETERIPROC glad_glTexParameteri = NULL;
PFNGLTEXPARAMETERFVPROC glad_glTexParameterfv = NULL;
PFNGLACTIVETEXTUREPROC glad_glActiveTexture = NULL;
PFNGLGENERATEMIPMAPPROC glad_glGenerateMipmap = NULL;

PFNGLGENBUFFERSPROC glad_glGenBuffers = NULL;
PFNGLDELETEBUFFERSPROC glad_glDeleteBuffers = NULL;
PFNGLBINDBUFFERPROC glad_glBindBuffer = NULL;
PFNGLBUFFERDATAPROC glad_glBufferData = NULL;
PFNGLBUFFERSUBDATAPROC glad_glBufferSubData = NULL;

PFNGLGENVERTEXARRAYSPROC glad_glGenVertexArrays = NULL;
PFNGLDELETEVERTEXARRAYSPROC glad_glDeleteVertexArrays = NULL;
PFNGLBINDVERTEXARRAYPROC glad_glBindVertexArray = NULL;
PFNGLENABLEVERTEXATTRIBARRAYPROC glad_glEnableVertexAttribArray = NULL;
PFNGLDISABLEVERTEXATTRIBARRAYPROC glad_glDisableVertexAttribArray = NULL;
PFNGLVERTEXATTRIBPOINTERPROC glad_glVertexAttribPointer = NULL;

PFNGLDRAWARRAYSPROC glad_glDrawArrays = NULL;
PFNGLDRAWElementsPROC glad_glDrawElements = NULL;

PFNGLCREATESHADERPROC glad_glCreateShader = NULL;
PFNGLDELETESHADERPROC glad_glDeleteShader = NULL;
PFNGLSHADERSOURCEPROC glad_glShaderSource = NULL;
PFNGLCOMPILESHADERPROC glad_glCompileShader = NULL;
PFNGLGETSHADERIVPROC glad_glGetShaderiv = NULL;
PFNGLGETSHADERINFOLOGPROC glad_glGetShaderInfoLog = NULL;

PFNGLCREATEPROGRAMPROC glad_glCreateProgram = NULL;
PFNGLDELETEPROGRAMPROC glad_glDeleteProgram = NULL;
PFNGLATTACHSHADERPROC glad_glAttachShader = NULL;
PFNGLLINKPROGRAMPROC glad_glLinkProgram = NULL;
PFNGLUSEPROGRAMPROC glad_glUseProgram = NULL;
PFNGLGETPROGRAMIVPROC glad_glGetProgramiv = NULL;
PFNGLGETPROGRAMINFOLOGPROC glad_glGetProgramInfoLog = NULL;

PFNGLGETUNIFORMLOCATIONPROC glad_glGetUniformLocation = NULL;
PFNGLUNIFORM1IPROC glad_glUniform1i = NULL;
PFNGLUNIFORM1FPROC glad_glUniform1f = NULL;
PFNGLUNIFORM2FPROC glad_glUniform2f = NULL;
PFNGLUNIFORM3FPROC glad_glUniform3f = NULL;
PFNGLUNIFORM4FPROC glad_glUniform4f = NULL;
PFNGLUNIFORMMATRIX4FVPROC glad_glUniformMatrix4fv = NULL;

PFNGLGENFRAMEBUFFERSPROC glad_glGenFramebuffers = NULL;
PFNGLDELETEFRAMEBUFFERSPROC glad_glDeleteFramebuffers = NULL;
PFNGLBINDFRAMEBUFFERPROC glad_glBindFramebuffer = NULL;
PFNGLFRAMEBUFFERTEXTURE2DPROC glad_glFramebufferTexture2D = NULL;
PFNGLCHECKFRAMEBUFFERSTATUSPROC glad_glCheckFramebufferStatus = NULL;

int gladLoadGLLoader(GLADloadproc load) {
#ifdef _WIN32
    if (glad_libGL == NULL) {
        glad_libGL = LoadLibraryA("opengl32.dll");
    }
#endif
    glad_glViewport = (PFNGLVIEWPORTPROC)glad_get_proc(load, "glViewport");
    glad_glClearColor = (PFNGLCLEARCOLORPROC)glad_get_proc(load, "glClearColor");
    glad_glClear = (PFNGLCLEARPROC)glad_get_proc(load, "glClear");
    glad_glEnable = (PFNGLENABLEPROC)glad_get_proc(load, "glEnable");
    glad_glDisable = (PFNGLDISABLEPROC)glad_get_proc(load, "glDisable");
    glad_glDepthFunc = (PFNGLDEPTHFUNCPROC)glad_get_proc(load, "glDepthFunc");
    glad_glDepthMask = (PFNGLDEPTHMASKPROC)glad_get_proc(load, "glDepthMask");
    glad_glBlendFunc = (PFNGLBLENDFUNCPROC)glad_get_proc(load, "glBlendFunc");
    glad_glCullFace = (PFNGLCULLFACEPROC)glad_get_proc(load, "glCullFace");
    glad_glFrontFace = (PFNGLFRONTFACEPROC)glad_get_proc(load, "glFrontFace");
    glad_glLineWidth = (PFNGLLINEWIDTHPROC)glad_get_proc(load, "glLineWidth");
    glad_glPolygonOffset = (PFNGLPOLYGONOFFSETPROC)glad_get_proc(load, "glPolygonOffset");
    glad_glPolygonMode = (PFNGLPOLYGONMODEPROC)glad_get_proc(load, "glPolygonMode");
    glad_glGetString = (PFNGLGETSTRINGPROC)glad_get_proc(load, "glGetString");
    glad_glGetError = (PFNGLGETERRORPROC)glad_get_proc(load, "glGetError");
    glad_glDrawBuffer = (PFNGLDRAWBUFFERPROC)glad_get_proc(load, "glDrawBuffer");
    glad_glReadBuffer = (PFNGLREADBUFFERPROC)glad_get_proc(load, "glReadBuffer");
    glad_glReadPixels = (PFNGLREADPIXELSPROC)glad_get_proc(load, "glReadPixels");

    glad_glGenTextures = (PFNGLGENTEXTURESPROC)glad_get_proc(load, "glGenTextures");
    glad_glDeleteTextures = (PFNGLDELETETEXTURESPROC)glad_get_proc(load, "glDeleteTextures");
    glad_glBindTexture = (PFNGLBINDTEXTUREPROC)glad_get_proc(load, "glBindTexture");
    glad_glTexImage2D = (PFNGLTEXIMAGE2DPROC)glad_get_proc(load, "glTexImage2D");
    glad_glTexParameteri = (PFNGLTEXPARAMETERIPROC)glad_get_proc(load, "glTexParameteri");
    glad_glTexParameterfv = (PFNGLTEXPARAMETERFVPROC)glad_get_proc(load, "glTexParameterfv");
    glad_glActiveTexture = (PFNGLACTIVETEXTUREPROC)glad_get_proc(load, "glActiveTexture");
    glad_glGenerateMipmap = (PFNGLGENERATEMIPMAPPROC)glad_get_proc(load, "glGenerateMipmap");

    glad_glGenBuffers = (PFNGLGENBUFFERSPROC)glad_get_proc(load, "glGenBuffers");
    glad_glDeleteBuffers = (PFNGLDELETEBUFFERSPROC)glad_get_proc(load, "glDeleteBuffers");
    glad_glBindBuffer = (PFNGLBINDBUFFERPROC)glad_get_proc(load, "glBindBuffer");
    glad_glBufferData = (PFNGLBUFFERDATAPROC)glad_get_proc(load, "glBufferData");
    glad_glBufferSubData = (PFNGLBUFFERSUBDATAPROC)glad_get_proc(load, "glBufferSubData");

    glad_glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)glad_get_proc(load, "glGenVertexArrays");
    glad_glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC)glad_get_proc(load, "glDeleteVertexArrays");
    glad_glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)glad_get_proc(load, "glBindVertexArray");
    glad_glEnableVertexAttribArray = (PFNGLENABLEVERTEXATTRIBARRAYPROC)glad_get_proc(load, "glEnableVertexAttribArray");
    glad_glDisableVertexAttribArray = (PFNGLDISABLEVERTEXATTRIBARRAYPROC)glad_get_proc(load, "glDisableVertexAttribArray");
    glad_glVertexAttribPointer = (PFNGLVERTEXATTRIBPOINTERPROC)glad_get_proc(load, "glVertexAttribPointer");

    glad_glDrawArrays = (PFNGLDRAWARRAYSPROC)glad_get_proc(load, "glDrawArrays");
    glad_glDrawElements = (PFNGLDRAWElementsPROC)glad_get_proc(load, "glDrawElements");

    glad_glCreateShader = (PFNGLCREATESHADERPROC)glad_get_proc(load, "glCreateShader");
    glad_glDeleteShader = (PFNGLDELETESHADERPROC)glad_get_proc(load, "glDeleteShader");
    glad_glShaderSource = (PFNGLSHADERSOURCEPROC)glad_get_proc(load, "glShaderSource");
    glad_glCompileShader = (PFNGLCOMPILESHADERPROC)glad_get_proc(load, "glCompileShader");
    glad_glGetShaderiv = (PFNGLGETSHADERIVPROC)glad_get_proc(load, "glGetShaderiv");
    glad_glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)glad_get_proc(load, "glGetShaderInfoLog");

    glad_glCreateProgram = (PFNGLCREATEPROGRAMPROC)glad_get_proc(load, "glCreateProgram");
    glad_glDeleteProgram = (PFNGLDELETEPROGRAMPROC)glad_get_proc(load, "glDeleteProgram");
    glad_glAttachShader = (PFNGLATTACHSHADERPROC)glad_get_proc(load, "glAttachShader");
    glad_glLinkProgram = (PFNGLLINKPROGRAMPROC)glad_get_proc(load, "glLinkProgram");
    glad_glUseProgram = (PFNGLUSEPROGRAMPROC)glad_get_proc(load, "glUseProgram");
    glad_glGetProgramiv = (PFNGLGETPROGRAMIVPROC)glad_get_proc(load, "glGetProgramiv");
    glad_glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC)glad_get_proc(load, "glGetProgramInfoLog");

    glad_glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)glad_get_proc(load, "glGetUniformLocation");
    glad_glUniform1i = (PFNGLUNIFORM1IPROC)glad_get_proc(load, "glUniform1i");
    glad_glUniform1f = (PFNGLUNIFORM1FPROC)glad_get_proc(load, "glUniform1f");
    glad_glUniform2f = (PFNGLUNIFORM2FPROC)glad_get_proc(load, "glUniform2f");
    glad_glUniform3f = (PFNGLUNIFORM3FPROC)glad_get_proc(load, "glUniform3f");
    glad_glUniform4f = (PFNGLUNIFORM4FPROC)glad_get_proc(load, "glUniform4f");
    glad_glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC)glad_get_proc(load, "glUniformMatrix4fv");

    glad_glGenFramebuffers = (PFNGLGENFRAMEBUFFERSPROC)glad_get_proc(load, "glGenFramebuffers");
    glad_glDeleteFramebuffers = (PFNGLDELETEFRAMEBUFFERSPROC)glad_get_proc(load, "glDeleteFramebuffers");
    glad_glBindFramebuffer = (PFNGLBINDFRAMEBUFFERPROC)glad_get_proc(load, "glBindFramebuffer");
    glad_glFramebufferTexture2D = (PFNGLFRAMEBUFFERTEXTURE2DPROC)glad_get_proc(load, "glFramebufferTexture2D");
    glad_glCheckFramebufferStatus = (PFNGLCHECKFRAMEBUFFERSTATUSPROC)glad_get_proc(load, "glCheckFramebufferStatus");

    return (glad_glViewport != NULL && glad_glCreateProgram != NULL && glad_glBindVertexArray != NULL) ? 1 : 0;
}
