
#ifndef _TG_ERROR_H
#define _TG_ERROR_H

#include "headers.h"
#include <string>

namespace TomGine{

/** @brief Check for OpenGL errors with a message prefix. \n
 * 	Note that only the first error since the last call of tgCheckError will be reported by OpenGL.
 * 	This might lead to wrong prefix messages.
 *  @param pre_msg	Message prefix.
 *  @param detailed	Enable/Disable detailed description of OpenGL errors. */
GLenum tgCheckError(std::string pre_msg, bool detailed=false);

/** @brief Check for OpenGL frame-buffer errors with a prefix message.
 *  @param target usually GL_FRAMEBUFFER (see glCheckFramebufferStatus in OpenGL spec.).
 *  @param pre_msg	message prefix. */
GLenum tgCheckFBError(GLenum target, std::string pre_msg);

} // namespace TomGine

#endif
