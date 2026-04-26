// SPDX-License-Identifier: GPL-2.0-or-later

#include "TrajectoryTokenizerTokenOps.hpp"

namespace {

std::string
FlipSignedValueString(std::string value)
{
  if (value == "+")
    return "-";
  if (value == "-")
    return "+";
  if (value == "0")
    return "0";

  if (!value.empty() && value[0] == '-')
    return value.substr(1);

  if (!value.empty() && value[0] == '+')
    return "-" + value.substr(1);

  return "-" + value;
}

bool
TryGetFieldValue(const std::string &token,
                 const std::string &prefix,
                 std::string &out)
{
  const auto pos = token.find(prefix);
  if (pos == std::string::npos)
    return false;

  const auto begin = pos + prefix.size();
  if (begin >= token.size())
    return false;

  auto end = begin;
  while (end < token.size() && token[end] != ',' && token[end] != ')' && token[end] != '|')
    ++end;

  if (end <= begin)
    return false;

  out = token.substr(begin, end - begin);
  return true;
}

void
ReplaceFieldValue(std::string &token,
                  const std::string &prefix,
                  const std::string &value)
{
  const auto pos = token.find(prefix);
  if (pos == std::string::npos)
    return;

  const auto begin = pos + prefix.size();
  if (begin >= token.size())
    return;

  auto end = begin;
  while (end < token.size() && token[end] != ',' && token[end] != ')' && token[end] != '|')
    ++end;

  token.replace(begin, end - begin, value);
}

} // namespace

namespace MultiAircraft {

std::string
CanonicalizeQTC3DTokenForSwappedPair(const std::string &in)
{
  std::string out = in;

  std::string z, l;
  if (TryGetFieldValue(out, "z=", z))
    ReplaceFieldValue(out, "z=", FlipSignedValueString(z));
  if (TryGetFieldValue(out, "l=", l))
    ReplaceFieldValue(out, "l=", FlipSignedValueString(l));

  std::string bA, bB;
  const bool has_ba = TryGetFieldValue(out, "bA=", bA);
  const bool has_bb = TryGetFieldValue(out, "bB=", bB);
  if (has_ba && has_bb)
  {
    ReplaceFieldValue(out, "bA=", bB);
    ReplaceFieldValue(out, "bB=", bA);
  }

  std::string dbA, dbB;
  const bool has_dba = TryGetFieldValue(out, "dbA=", dbA);
  const bool has_dbb = TryGetFieldValue(out, "dbB=", dbB);
  if (has_dba && has_dbb)
  {
    ReplaceFieldValue(out, "dbA=", dbB);
    ReplaceFieldValue(out, "dbB=", dbA);
  }

  return out;
}

std::string
MirrorQTC3DTokenHandedness(const std::string &in)
{
  std::string out = in;

  std::string l;
  if (TryGetFieldValue(out, "l=", l))
    ReplaceFieldValue(out, "l=", FlipSignedValueString(l));

  std::string bA, bB;
  if (TryGetFieldValue(out, "bA=", bA))
    ReplaceFieldValue(out, "bA=", FlipSignedValueString(bA));
  if (TryGetFieldValue(out, "bB=", bB))
    ReplaceFieldValue(out, "bB=", FlipSignedValueString(bB));

  std::string dbA, dbB;
  if (TryGetFieldValue(out, "dbA=", dbA))
    ReplaceFieldValue(out, "dbA=", FlipSignedValueString(dbA));
  if (TryGetFieldValue(out, "dbB=", dbB))
    ReplaceFieldValue(out, "dbB=", FlipSignedValueString(dbB));

  return out;
}

} // namespace MultiAircraft
