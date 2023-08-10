// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "DebugReplayFile.hpp"
#include "IGC/IGCExtensions.hpp"
#include "IGC/IGCFRInfo.hpp"
#include "IGC/IGCHeader.hpp"
#include "io/FileLineReader.hpp"

struct IGCFix;

class DebugReplayIGC : public DebugReplayFile {
  IGCExtensions extensions;
  IGCFRInfo fr_info;
  IGCHeader header;
  double h_acc = 5.0;

private:
  DebugReplayIGC(FileLineReaderA *_reader)
    : DebugReplayFile(_reader) {
    extensions.clear();
    fr_info.clear();
  }

public:
  virtual bool Next() override;
  virtual bool Rewind() override;

  static DebugReplay *Create(Path input_file);
  std::string GetTypeInfo() const override;
  std::string GetIdentifier() const override;
  double GetHAccuracy() const override {
    return h_acc;
  }

protected:
  void CopyFromFix(const IGCFix &fix);
};
