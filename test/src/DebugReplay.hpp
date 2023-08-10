// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "Computer/BasicComputer.hpp"
#include "Computer/FlyingComputer.hpp"
#include "Engine/GlideSolvers/GlidePolar.hpp"
#include "time/WrapClock.hpp"
#include "system/Args.hpp"
#include "Atmosphere/Pressure.hpp"
#include <string>

class DebugReplay {
protected:
  GlidePolar glide_polar;

  BasicComputer computer;
  FlyingComputer flying_computer;

  /**
   * Raw values parsed from the NMEA/IGC file.
   */
  NMEAInfo raw_basic;

  /**
   * A copy of #raw_basic with #BasicComputer changes.
   */
  MoreData computed_basic;

  /**
   * The #computed_basic value from the previous iteration.
   */
  MoreData last_basic;

  DerivedInfo calculated;

  WrapClock wrap_clock;

  AtmosphericPressure qnh;

public:
  DebugReplay();
  virtual ~DebugReplay();
  virtual bool Rewind() {
    return false;
  };
  void Reset();

  virtual bool Next() = 0;
  virtual std::string GetTypeInfo() const = 0;
  virtual std::string GetIdentifier() const = 0;
  virtual double GetHAccuracy() const = 0;

  /* Return a detail level for this fix - only used for skylines */
  virtual int Level() const {
    return 0;
  }

  const MoreData &Basic() const {
    return computed_basic;
  }

  const MoreData &LastBasic() const {
    return last_basic;
  }

  const DerivedInfo &Calculated() const {
    return calculated;
  }

  DerivedInfo &SetCalculated() {
    return calculated;
  }

  FlyingComputer &SetFlyingComputer() {
    return flying_computer;
  }

  void SetQNH(const AtmosphericPressure _qnh) {
    qnh = _qnh;
  }

protected:
  void Compute();
};

DebugReplay *
CreateDebugReplay(Args &args);
