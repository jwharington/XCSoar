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

#include <cstdint>
#include "util/Compiler.h"
#include "Math/FastTrig.hpp"
#include "VarioSettings.hpp"
#include "util/Clamp.hpp"

typedef enum
{
  TONE_CONTINUOUS = 0,
  TONE_SILENT,
  TONE_SHORT_BEEP,
  TONE_LONG_BEEP
} ToneType;

class DDSCommon {
 public:
  static void init(const VarioSoundSettings& settings);
  static bool silent;

  // tone synthesiser options
#define TONE_RANGE 128
#define ENVELOPE_WIDTH 256
#define ENVELOPE_MAX_VOLUME 256
#define TONE_WIDTH_BITS 12
  // (4096 samples in ISINETABLE)

  static uint16_t tone_freq_table[2*TONE_RANGE];
  static uint8_t tone_envelope_table[ENVELOPE_WIDTH];

  // interpolation
  static uint16_t sample_count_last;
  static uint16_t dt_sample;
  static int tone_last;
  static int tone_now;
  static unsigned sample_rate;
  static uint16_t sample_counter;

  static int clamp_tone(const int tone)
  {
    return Clamp(tone, 0, 2*TONE_RANGE-1);
  }

  static int sine(const uint16_t phase) {
    // ISINETABLE is only 10 bit
    return ISINETABLE[phase>>(16-TONE_WIDTH_BITS)] <<5;
  }

 private:
  static void init_tone_envelope_table();

  /**
   * Convert a value to a tone frequency.
   *
   * @param i value in range [-TONE_RANGE:TONE_RANGE-1]
   */
  gcc_const
  static double value_to_frequency(const int i, const VarioSoundSettings& settings);
};

struct DDSSynth: public DDSCommon {

#define MASTER_MAX_VOLUME 128

  void init() {
    phase_step = tone_freq_table[TONE_RANGE];
    clear_deadband();
  }

  int get_sample()
  {
    wave_advance();
    if (gcc_unlikely(wave_start())) {
      update();
    }
    return envelope_volume*sine(phase_counter);
  }

  void clear_deadband() {
    deadband_min = 2*TONE_RANGE-1;
    deadband_max = 0;
  }

  void set_deadband(const int imin, const int imax) {
    deadband_min = clamp_tone(imin);
    deadband_max = clamp_tone(imax);
  }

 private:
  ToneType type = TONE_SILENT;
  uint32_t phase_step = 0;
  static uint32_t phase_counter;
  uint32_t envelope_volume = 0;
  unsigned deadband_max = 0;
  unsigned deadband_min = 2*TONE_RANGE-1;

  bool wave_start() const {
    return (phase_counter & 0xFFFF) <= phase_step;
  }
  void wave_advance() {
    phase_counter += phase_step;
    sample_counter++;
  }

  void update();
  ToneType calc_type(const unsigned tone_this) const;
};
