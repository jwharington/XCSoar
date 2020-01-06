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

#include "ToneSynthesiser.hpp"
#include "Math/FastTrig.hpp"
#include "util/Macros.hpp"
#include "util/Clamp.hpp"

#include <cassert>

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

//////////////////////////////////////////////////////////////////////////////////////////

int16_t ToneSynthesiser::get_sample()
{
  return
      (dds_synthesiser.get_sample()*master_volume /* + dither.update() */)
      /(ENVELOPE_MAX_VOLUME*MASTER_MAX_VOLUME);
}

void ToneSynthesiser::init(const VarioSoundSettings& settings)
{
  master_volume = settings.volume;
  DDSCommon::init(settings);

  // now initialise player
  dds_synthesiser.init();
}

void
ToneSynthesiser::Synthesise(int16_t *buffer, size_t n)
{
  int16_t* p = buffer;
  const int16_t* e = p + n;
  while (p<e) {
    *p++ = get_sample();
  }
}

void
ToneSynthesiser::update_sample(const int value)
{
  tone_last = tone_now;

  dt_sample = sample_counter - sample_count_last;
  sample_count_last = sample_counter;

  tone_now = DDSCommon::clamp_tone(value);
  silent = false; // start playing

  if ((dt_sample > sample_rate*2) || (!dt_sample)) {
    // too slow or too fast, don't interpolate
    dt_sample = sample_rate;
    tone_last = tone_now;
  }
}

void ToneSynthesiser::SetSilence()
{
  silent = true;
}

void ToneSynthesiser::set_deadband(const int imin, const int imax)
{
  dds_synthesiser.set_deadband(imin, imax);
}

void ToneSynthesiser::clear_deadband()
{
  dds_synthesiser.clear_deadband();
}
