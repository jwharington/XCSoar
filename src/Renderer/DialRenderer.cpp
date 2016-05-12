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

#include "DialRenderer.hpp"
#include "Screen/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "Look/DialLook.hpp"
#include "util/Macros.hpp"
#include "Math/Screen.hpp"

#include <algorithm>

struct DialRendererLayout {
  double f0;
  double f1;
};

static constexpr DialRendererLayout layout[] =
{
  { 0.00, 1.00},
  { 0.25, 0.75},
  { 0.25, 1.00},
  { 0.50, 1.00},
  { 0.50, 1.25},
  { 0.75, 1.25},
  { 0.75, 1.50},
  { 0.00, 0.50},
  { 0.00, 0.75},
  { 0.00, 1.00},
};

static_assert(ARRAY_SIZE(layout) == DialStyle::COUNT,
              "Wrong dial renderer layout size");

DialRenderer::DialRenderer(const DialLook &_look,
                           const bool& _inverse,
                           const DialStyle& style):look(_look),
                                                   inverse(_inverse)
{
  assert(style < DialStyle::COUNT);
  a0 = Angle::FullCircle() * layout[style].f0;
  a1 = Angle::FullCircle() * layout[style].f1;
}

void
DialRenderer::DrawOutline(Canvas &canvas, const int radius, const PixelPoint &center)
{
  // draw outline arc
  const Pen pen_0(Layout::ScalePenWidth(1), COLOR_GRAY);
  canvas.Select(pen_0);
  canvas.DrawArc(center, radius, a0, a1);
}

void
DialRenderer::DrawActiveSegment(Canvas &canvas, const int radius, const PixelPoint &center, const Angle &az, const Angle &av)
{
  canvas.Select(look.brush_value);
  canvas.SelectNullPen();

  canvas.DrawSegment(center, radius, az, av);
  const Pen pen(Layout::ScalePenWidth(2), inverse ? COLOR_WHITE : COLOR_BLACK);
  canvas.Select(pen);
  canvas.DrawArc(center, radius, az, av);
}

void
DialRenderer::DrawPointer(Canvas &canvas, const int radius, const PixelPoint &center, const Angle &av)
{
  const Pen pen(Layout::ScalePenWidth(2), inverse ? COLOR_WHITE : COLOR_BLACK);
  canvas.Select(pen);
  BulkPixelPoint arrow[4] = { { 0, -radius*1024/Layout::Scale(1024)-2 }, { -2, 0 }, {0, 2}, { 2, 0 } };
  PolygonRotateShift(arrow, ARRAY_SIZE(arrow), center, av);
  if (inverse)
    canvas.SelectWhiteBrush();
  else
    canvas.SelectBlackBrush();
  canvas.DrawPolygon(arrow, ARRAY_SIZE(arrow));
}

void
DialRenderer::Draw(double value,
                   Canvas &canvas,
                   const PixelRect &rc)
{
  const int radius = std::min(rc.GetWidth(), rc.GetHeight()) / 2 -
                     Layout::Scale(3);
  PixelPoint center;
  center.x = (rc.left + rc.right) / 2;
  center.y = (rc.bottom + rc.top) / 2;

  DrawOutline(canvas, radius, center);

  assert(max>min);
  assert(max>= zero);
  assert(zero>= min);

  const double rel = std::min(1.0,std::max((value-min)/(max-min),0.0));
  const double zrel = std::min(1.0,std::max((zero-min)/(max-min),0.0));
  const Angle av = a0+(a1-a0)*rel;
  const Angle az = a0+(a1-a0)*zrel;

  if ((av-az).Absolute().Degrees() > 10.0) {
    if (rel>zrel) {
      DrawActiveSegment(canvas, radius, center, az, av);
    } else if (rel<zrel) {
      DrawActiveSegment(canvas, radius, center, av, az);
    }
  }

  DrawPointer(canvas, radius, center, av);
}
