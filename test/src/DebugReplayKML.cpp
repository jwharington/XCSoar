// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "DebugReplayKML.hpp"

#include "Geo/GeoPoint.hpp"
#include "io/FileLineReader.hpp"
#include "io/ZipArchive.hpp"
#include "io/ZipLineReader.hpp"
#include "system/Path.hpp"
#include "util/StringCompare.hxx"
#include "util/StringStrip.hxx"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

static std::string_view
Trim(std::string_view s) noexcept
{
  return Strip(s);
}

static bool
ParseUInt(std::string_view s, unsigned &value) noexcept
{
  if (s.empty())
    return false;

  unsigned v = 0;
  for (const char ch : s)
  {
    if (ch < '0' || ch > '9')
      return false;

    v = v * 10u + unsigned(ch - '0');
  }

  value = v;
  return true;
}

static bool
ParseKmlWhenUTC(std::string_view text, BrokenDateTime &dt) noexcept
{
  text = Trim(text);
  if (text.size() < 20 || text.back() != 'Z')
    return false;

  text.remove_suffix(1); // remove trailing 'Z'

  const std::size_t dot = text.find('.');
  if (dot != std::string_view::npos)
    text = text.substr(0, dot);

  if (text.size() != 19 || text[4] != '-' || text[7] != '-' ||
      text[10] != 'T' || text[13] != ':' || text[16] != ':')
    return false;

  unsigned year, month, day, hour, minute, second;
  if (!ParseUInt(text.substr(0, 4), year) ||
      !ParseUInt(text.substr(5, 2), month) ||
      !ParseUInt(text.substr(8, 2), day) ||
      !ParseUInt(text.substr(11, 2), hour) ||
      !ParseUInt(text.substr(14, 2), minute) ||
      !ParseUInt(text.substr(17, 2), second))
    return false;

  dt = BrokenDateTime(year, month, day, hour, minute, second);
  return dt.IsPlausible();
}

static bool
ParseGxCoord(std::string_view text, GeoPoint &location, double &altitude) noexcept
{
  std::string buffer(Trim(text));
  char *p = buffer.data();

  char *end = nullptr;
  const double lon = std::strtod(p, &end);
  if (end == p)
    return false;

  p = end;
  const double lat = std::strtod(p, &end);
  if (end == p)
    return false;

  p = end;
  const double alt = std::strtod(p, &end);
  if (end == p)
    return false;

  location = GeoPoint(Angle::Degrees(lon), Angle::Degrees(lat));
  altitude = alt;
  return true;
}

static std::vector<std::string_view>
ExtractTagValues(std::string_view text, const std::string_view open_tag,
                 const std::string_view close_tag)
{
  std::vector<std::string_view> values;

  std::size_t pos = 0;
  while (true)
  {
    const std::size_t open = text.find(open_tag, pos);
    if (open == std::string_view::npos)
      break;

    const std::size_t value_begin = open + open_tag.size();
    const std::size_t close = text.find(close_tag, value_begin);
    if (close == std::string_view::npos)
      throw std::runtime_error("malformed KML: unterminated tag");

    values.push_back(text.substr(value_begin, close - value_begin));
    pos = close + close_tag.size();
  }

  return values;
}

static std::vector<DebugReplayKML::Fix>
ParseKmlGxTrack(const std::string &kml_text)
{
  std::vector<DebugReplayKML::Fix> fixes;

  std::size_t pos = 0;
  while (true)
  {
    const std::size_t track_start = kml_text.find("<gx:Track", pos);
    if (track_start == std::string::npos)
      break;

    const std::size_t track_open_end = kml_text.find('>', track_start);
    if (track_open_end == std::string::npos)
      throw std::runtime_error("malformed KML: gx:Track tag not closed");

    const std::size_t track_end = kml_text.find("</gx:Track>", track_open_end + 1);
    if (track_end == std::string::npos)
      throw std::runtime_error("malformed KML: missing </gx:Track>");

    const std::string_view track_body(kml_text.data() + track_open_end + 1,
                                      track_end - track_open_end - 1);

    const auto when_values = ExtractTagValues(track_body, "<when>", "</when>");
    const auto coord_values = ExtractTagValues(track_body, "<gx:coord>", "</gx:coord>");

    if (when_values.empty() || coord_values.empty())
      throw std::runtime_error("KML gx:Track must contain <when> and <gx:coord> entries");

    if (when_values.size() != coord_values.size())
      throw std::runtime_error("KML gx:Track has mismatched <when> and <gx:coord> counts");

    for (std::size_t i = 0; i < when_values.size(); ++i)
    {
      DebugReplayKML::Fix fix;
      if (!ParseKmlWhenUTC(when_values[i], fix.date_time_utc))
        throw std::runtime_error("invalid KML <when>: expected UTC timestamp like YYYY-MM-DDThh:mm:ssZ");

      if (!ParseGxCoord(coord_values[i], fix.location, fix.gps_altitude))
        throw std::runtime_error("invalid KML <gx:coord>: expected 'lon lat alt'");

      fixes.push_back(fix);
    }

    pos = track_end + 11;
  }

  if (fixes.empty())
    throw std::runtime_error("no gx:Track data found in KML file");

  return fixes;
}

DebugReplayKML::DebugReplayKML(std::vector<Fix> &&_fixes) noexcept
  : fixes(std::move(_fixes))
{
}

static std::string
ReadTextFile(Path path)
{
  FileLineReaderA reader(path);
  std::string content;

  while (const char *line = reader.ReadLine())
  {
    content.append(line);
    content.push_back('\n');
  }

  if (content.empty())
    throw std::runtime_error("empty KML input");

  return content;
}

static std::string
ReadTextFromKmz(Path path)
{
  ZipArchive archive(path);

  std::string kml_entry;
  if (archive.Exists("doc.kml"))
  {
    kml_entry = "doc.kml";
  }
  else
  {
    while (true)
    {
      const std::string name = archive.NextName();
      if (name.empty())
        break;

      if (StringEndsWithIgnoreCase(name.c_str(), ".kml"))
      {
        kml_entry = name;
        break;
      }
    }
  }

  if (kml_entry.empty())
    throw std::runtime_error("KMZ contains no .kml entry");

  ZipLineReaderA reader(archive.get(), kml_entry.c_str());
  std::string content;

  while (const char *line = reader.ReadLine())
  {
    content.append(line);
    content.push_back('\n');
  }

  if (content.empty())
    throw std::runtime_error("selected KML entry in KMZ is empty");

  return content;
}

DebugReplay *
DebugReplayKML::Create(Path input_file)
{
  try
  {
    const std::string content = input_file.EndsWithIgnoreCase(".kmz")
                                    ? ReadTextFromKmz(input_file)
                                    : ReadTextFile(input_file);

    auto fixes = ParseKmlGxTrack(content);
    return new DebugReplayKML(std::move(fixes));
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to parse KML/KMZ track: " << e.what() << "\n";
    return nullptr;
  }
}

bool
DebugReplayKML::Rewind()
{
  Reset();
  index = 0;
  return true;
}

bool
DebugReplayKML::Next()
{
  last_basic = computed_basic;

  if (index >= fixes.size())
  {
    if (computed_basic.time_available)
      flying_computer.Finish(calculated.flight, computed_basic.time);

    return false;
  }

  CopyFromFix(fixes[index++]);
  Compute();
  return true;
}

void
DebugReplayKML::CopyFromFix(const Fix &fix)
{
  NMEAInfo &basic = raw_basic;

  basic.clock = basic.time = TimeStamp{fix.date_time_utc.GetTime().DurationSinceMidnight()};
  basic.time_available.Update(basic.clock);
  basic.date_time_utc = fix.date_time_utc;

  basic.alive.Update(basic.clock);
  basic.location = fix.location;
  basic.location_available.Update(basic.clock);

  basic.gps_altitude = fix.gps_altitude;
  basic.gps_altitude_available.Update(basic.clock);

  /* KML has no barometric altitude; mirror GPS so downstream blending
     logic has a sane value without synthesising timing. */
  basic.pressure_altitude = fix.gps_altitude;
  basic.pressure_altitude_available.Update(basic.clock);
}

std::string
DebugReplayKML::GetTypeInfo() const
{
  return std::string("KML gx:Track");
}

std::string
DebugReplayKML::GetIdentifier() const
{
  return std::string("KML");
}

double
DebugReplayKML::GetHAccuracy() const
{
  return h_acc;
}
