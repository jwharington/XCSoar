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

#ifndef XCSOAR_AUDIO_VARIO_SYNTHESISER_HPP
#define XCSOAR_AUDIO_VARIO_SYNTHESISER_HPP

#include "ToneSynthesiser.hpp"
#include "thread/Mutex.hxx"
#include "util/Compiler.h"
#include "Dither.hpp"
#include "VarioSettings.hpp"

/**
 * This class generates vario sound.
 */
class VarioSynthesiser final : public ToneSynthesiser {
  /**
   * This mutex protects all atttributes below.  It is locked
   * automatically by all public methods.
   */
  Mutex mutex;

  VarioSoundSettings settings;

public:
  explicit VarioSynthesiser(unsigned sample_rate)
    :ToneSynthesiser(sample_rate) {
    VarioSoundSettings _settings;
    _settings.SetDefaults();
    init(_settings);
  }

  /**
   * Update the vario value.  This calculates a new tone frequency and
   * a new "silence" rate (for positive vario values).
   *
   * @param vario the current vario value [m/s]
   */
  void SetVario(const double vario, const double netto);

  /**
   * Produce silence from now on.
   */
  void SetSilence();

  /* methods from class PCMSynthesiser */
  virtual void Synthesise(int16_t *buffer, size_t n) override;

 public:
  void init(const VarioSoundSettings& _settings);
};

#endif
