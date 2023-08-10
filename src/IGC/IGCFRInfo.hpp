// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

struct IGCFRInfo {
  int geoid_correction;
  char fr_type[80];
  char fw_version[80];
  void CheckCorrection();
  void clear();
};

bool IGCFRInfoDB_init(const char* file_path);
