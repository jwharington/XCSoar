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

#ifndef XCSOAR_AUDIO_TONE_SYNTHESISER_HPP
#define XCSOAR_AUDIO_TONE_SYNTHESISER_HPP

#include "PCMSynthesiser.hpp"
#include "util/Compiler.h"

#include "DDSSynth.hpp"
#include "Dither.hpp"

/**
 * This class generates tones with a sine wave.
 */
class ToneSynthesiser : public PCMSynthesiser, private DDSCommon {
 public:
  explicit ToneSynthesiser(const unsigned _sample_rate) {
    sample_rate = _sample_rate;
  }

  void init(const VarioSoundSettings& settings);

  unsigned GetSampleRate() const override {
    return sample_rate;
  }

  /* methods from class PCMSynthesiser */
  virtual void Synthesise(int16_t *buffer, size_t n) override;

  void SetSilence();

  void update_sample(const int value);
  void set_deadband(const int imin, const int imax);
  void clear_deadband();

 private:
  int16_t get_sample();
  Dither dither;
  uint8_t master_volume = 0;
  DDSSynth dds_synthesiser;
};

#endif
