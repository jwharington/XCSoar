// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "DebugReplayFile.hpp"
#include "IGC/IGCExtensions.hpp"
#include "io/FileLineReader.hpp"

struct IGCFix;

class DebugReplayIGC : public DebugReplayFile {
  IGCExtensions extensions;
  double h_acc = 5.0;

private:
  DebugReplayIGC(FileLineReaderA *_reader)
    : DebugReplayFile(_reader) {
    extensions.clear();
  }

public:
  virtual bool Next() override;
  virtual bool Rewind() override;

  static DebugReplay *Create(Path input_file);
  double GetHAccuracy() const override {
    return h_acc;
  }

protected:
  void CopyFromFix(const IGCFix &fix);
};
