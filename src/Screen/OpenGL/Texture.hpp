/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_SCREEN_OPENGL_TEXTURE_HPP
#define XCSOAR_SCREEN_OPENGL_TEXTURE_HPP

#include "Asset.hpp"

#include <SDL.h>

#ifdef ANDROID
#include <GLES/gl.h>
#else
#include <SDL_opengl.h>
#endif

/**
 * This class represents an OpenGL texture.
 */
class GLTexture {
  GLuint id;
  unsigned width, height;

public:
  GLTexture(SDL_Surface *surface) {
    init();
    load(surface);
  }

  ~GLTexture() {
    glDeleteTextures(1, &id);
  }

  unsigned get_width() const {
    return width;
  }

  unsigned get_height() const {
    return height;
  }

protected:
  void init() {
    glGenTextures(1, &id);
    bind();
    configure();
  }

  static inline void configure() {
    if (is_embedded()) {
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    } else {
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
  }

  void load(SDL_Surface *surface);

public:
  void bind() {
    glBindTexture(GL_TEXTURE_2D, id);
  }

  void update(SDL_Surface *surface);

  void draw(int x_offset, int y_offset,
            int dest_x, int dest_y,
            unsigned dest_width, unsigned dest_height,
            int src_x, int src_y,
            unsigned src_width, unsigned src_height) const;

  void draw(int x_offset, int y_offset,
            int dest_x, int dest_y) const {
    draw(x_offset, y_offset,
         dest_x, dest_y, width, height,
         0, 0, width, height);
  }
};

#endif
