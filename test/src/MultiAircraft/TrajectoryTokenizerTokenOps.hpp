// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <string>

namespace MultiAircraft {

std::string
CanonicalizeQTC3DTokenForSwappedPair(const std::string &in);

std::string
MirrorQTC3DTokenHandedness(const std::string &in);

} // namespace MultiAircraft
