/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
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

#ifndef DIAL_RENDERER_HPP
#define DIAL_RENDERER_HPP

#include "Math/Angle.hpp"
#include "InfoBoxes/DialStyle.hpp"

struct PixelRect;
class Canvas;
struct DialLook;
struct PixelPoint;

class DialRenderer {
  const DialLook &look;
  const bool &inverse;
  Angle a0;
  Angle a1;

public:
  DialRenderer(const DialLook &_look, const bool& _inverse, const DialStyle& style);

  void Draw(double value, Canvas &canvas,
            const PixelRect &rc);
  double min;
  double max;
  double zero;

private:
  void DrawOutline(Canvas &canvas, const int radius, const PixelPoint &center);
  void DrawActiveSegment(Canvas &canvas, const int radius, const PixelPoint &center, const Angle &az, const Angle &av);
  void DrawPointer(Canvas &canvas, const int radius, const PixelPoint &center, const Angle &av);
};

#endif
