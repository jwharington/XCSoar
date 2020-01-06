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

#include "VarioSynthesiser.hpp"
#include "Math/FastMath.hpp"
#include "Math/FastTrig.hpp"

#include "util/Compiler.h"
#include "util/Macros.hpp"

#include <algorithm>
#include <cmath>

#include <cassert>

void VarioSynthesiser::init(const VarioSoundSettings& _settings)
{
  // initialise settings
  settings = _settings;
  ToneSynthesiser::init(settings);
}

static int vario_to_tone(const double vario)
{
  const double vario_max = 5.0;
  return lrintf(vario*TONE_RANGE/vario_max)+TONE_RANGE;
}

void
VarioSynthesiser::SetVario(const double vario, const double netto)
{
  const std::lock_guard<Mutex> lock(mutex);

  // reduce tone scale across deadband
  float v_i = vario;
  if (settings.dead_band_enabled) {
    if (netto > settings.max_dead) {
      v_i -= settings.max_dead/2;
    } else if (netto < settings.min_dead) {
      v_i -= settings.min_dead/2;
    }
    set_deadband(vario_to_tone(settings.min_dead), vario_to_tone(settings.max_dead));
  } else {
    clear_deadband();
  }

  update_sample(vario_to_tone(v_i));
}

void
VarioSynthesiser::SetSilence()
{
  const std::lock_guard<Mutex> lock(mutex);
  ToneSynthesiser::SetSilence();
}

void
VarioSynthesiser::Synthesise(int16_t *buffer, size_t n)
{
  const std::lock_guard<Mutex> lock(mutex);
  ToneSynthesiser::Synthesise(buffer, n);
}


