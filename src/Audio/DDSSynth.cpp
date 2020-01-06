/*
  Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2020 The XCSoar Project
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

#include "DDSSynth.hpp"
#include <algorithm>
#include <assert.h>

static constexpr int STEP_LENGTH = 32;
static constexpr int TONE_LENGTH = 192;
static constexpr int ATTACK_FACTOR = (ENVELOPE_MAX_VOLUME)/6;
static constexpr uint32_t CONTINUOUS_RAMP_WIDTH = ENVELOPE_WIDTH/16;
static constexpr uint32_t CONTINUOUS_RAMP_SLOPE = (ENVELOPE_MAX_VOLUME-1)/CONTINUOUS_RAMP_WIDTH;

bool DDSCommon::silent = true;
uint16_t DDSCommon::sample_count_last = 0;
uint16_t DDSCommon::dt_sample = 1000;
int DDSCommon::tone_last = TONE_RANGE;
int DDSCommon::tone_now = TONE_RANGE;
uint16_t DDSCommon::tone_freq_table[2*TONE_RANGE];
uint8_t DDSCommon::tone_envelope_table[ENVELOPE_WIDTH];
unsigned DDSCommon::sample_rate = 0;
uint16_t DDSCommon::sample_counter = 0;
uint32_t DDSSynth::phase_counter = -1;


ToneType DDSSynth::calc_type(const unsigned tone_this) const
{
  if (silent || ((tone_this < deadband_max) && (tone_this > deadband_min))) {
    return TONE_SILENT;
  } else if (tone_this >= TONE_RANGE) {
    return TONE_SHORT_BEEP;
  } else {
    return TONE_CONTINUOUS;
  }
}

void DDSSynth::update()
{
  // always update tone frequency (on phase overflow)
  const uint16_t dt = std::min(dt_sample, (uint16_t)(sample_counter - sample_count_last));
  const unsigned tone_this = DDSCommon::clamp_tone(((dt_sample-dt)* tone_last + dt * tone_now)/dt_sample);
  phase_step = tone_freq_table[tone_this];

  const ToneType intermediate_type = calc_type(tone_this);

  const uint8_t adsr_phase = (phase_counter>>16) & 0xFF;
  const uint8_t neg_adsr_phase = ~adsr_phase;

  // update type on adsr overflow
  if (!adsr_phase) {
    type = intermediate_type;
  }

  // update volume on phase overflow
  switch (type) {
    case TONE_SHORT_BEEP:
      envelope_volume = tone_envelope_table[adsr_phase];
      break;
    case TONE_CONTINUOUS:
    case TONE_LONG_BEEP:
      if (adsr_phase <= CONTINUOUS_RAMP_WIDTH) {
        // up-ramp
        envelope_volume = std::max(envelope_volume, adsr_phase*CONTINUOUS_RAMP_SLOPE);
      } else if ((intermediate_type != TONE_CONTINUOUS) && (neg_adsr_phase < CONTINUOUS_RAMP_WIDTH)) {
        // down-ramp only if moving away from continuous
        envelope_volume = neg_adsr_phase*CONTINUOUS_RAMP_SLOPE;
      }
      break;
    case TONE_SILENT:
    default:
      envelope_volume = 0;
      break;
  };
}


void DDSCommon::init_tone_envelope_table()
{
  for (int i=0; i<ENVELOPE_WIDTH; ++i) {
    uint16_t adsr_mag;
    if (i*ATTACK_FACTOR<ENVELOPE_MAX_VOLUME) {
      // attack phase
      adsr_mag = i*ATTACK_FACTOR;
    } else if (i<TONE_LENGTH-3*STEP_LENGTH) {
      // sustain
      adsr_mag = ENVELOPE_MAX_VOLUME-1;
    } else if (i<TONE_LENGTH) {
      // release
      float x = (TONE_LENGTH-i)/(3.0*STEP_LENGTH);
      adsr_mag = (ENVELOPE_MAX_VOLUME-1)*x*x;
    } else {
      // silence
      adsr_mag = 0;
    }
    tone_envelope_table[i] = adsr_mag;
  }
}


double
DDSCommon::value_to_frequency(const int i, const VarioSoundSettings& settings)
{
  if (i>=0) {
    const double p = settings.max_frequency/(double)settings.zero_frequency;
    const double t = i/(double)TONE_RANGE;
    return settings.zero_frequency*pow(p, t);
  } else {
    const double p = settings.zero_frequency/(double)settings.min_frequency;
    const double t = 1+i/(double)TONE_RANGE;
    return settings.min_frequency*pow(p, t);
  }
}

void DDSCommon::init(const VarioSoundSettings& settings)
{
  // initialise tone frequency table
  for (int i=0; i<TONE_RANGE*2; ++i) {
    const double f_hz = value_to_frequency(i-TONE_RANGE, settings);
    tone_freq_table[i] = round(f_hz*(0xFFFF)/sample_rate);
  }

  init_tone_envelope_table();

  silent = true;
}
