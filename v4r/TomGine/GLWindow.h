/**
 * @file GLWindow.h
 * @author Thomas Moerwald (Vienna University of Technology)
 * @date June 2010
 * @version 0.1
 * @brief Device Context for handling OpenGL windows in MS GLWindows.
 */

#ifndef _GL_WINDOW_
#define _GL_WINDOW_

#include "GLEvent.h"
#include "GLInput.h"

#ifdef WIN32
#include <windows.h>
#include <gl/glew.h>
#include <gl/gl.h>

#else

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <semaphore.h>

// Extend list of Buttons in X.h
#define Button6     6
#define Button7     7
#define Button8     8
#define Button9     9

#endif

/** @brief BLORT namespace for GLWindow */
namespace TomGine {

#ifdef WIN32
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif
class Event;

/** @brief Class GLWindow */
class GLWindow
{

public:
  /** @brief Construction of an OpenGL Window (Rendering Context)
   *   @param width Window and OpenGL viewport width in pixel
   *   @param height Window and OpenGL viewport height in pixel
   *   @param name Caption of the window in the titel bar */
  GLWindow(unsigned width = 320, unsigned height = 240, const char* name = "GLWindow",
      bool threaded = false, bool stereo = false);
  ~GLWindow();

  /** @brief Activate window for usage (set focus) */
  void Activate();

  /** @brief Update OpenGL GLWindow (Swap Buffers) */
  void Update();

  /** @brief Query input event 
   *   @return true if there are events in the event que
   *		@param Event defined in TMGLEvent.h */
  bool GetEvent(Event &event);
  void GetEventBlocking(Event &event);
  void UnBlockGetEvent();

#ifdef WIN32
  HWND gethWnd() const {return hWnd;}
#else
  inline GLuint GetFontBase()
  {
    return font_base;
  }
#endif

private:
#ifdef WIN32
  WNDCLASS wc;
  HWND hWnd;
  HDC hDC;
  HGLRC hRC;
  MSG msg;
#else
  Display *dpy;
  Window root;
  XVisualInfo *vi;
  XFontStruct *font_info;
  GLuint font_base;
  Colormap cmap;
  XSetWindowAttributes swa;
  Window glWin;
  Window btWin;
  Atom wmDelete;
  GLXContext glc;
  XWindowAttributes gwa;
  bool threaded;
#endif

  void init(unsigned int width, unsigned int height, const char* name, bool threaded = false,
      bool stereo = false);
  void quit();

};

} /* namespace */

#endif /* _GL_WINDOW_ */
