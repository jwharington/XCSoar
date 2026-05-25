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

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

struct InputSpec {
  std::string path;
  std::string selector;
};

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

static bool
ParsePointCoordinates(std::string_view text, GeoPoint &location, double &altitude) noexcept
{
  std::string buffer(Trim(text));

  char *end = nullptr;
  const double lon = std::strtod(buffer.c_str(), &end);
  if (end == buffer.c_str() || *end != ',')
    return false;

  const char *p = end + 1;
  const double lat = std::strtod(p, &end);
  if (end == p)
    return false;

  altitude = 0;
  if (*end == ',')
  {
    p = end + 1;
    altitude = std::strtod(p, &end);
    if (end == p)
      return false;
  }

  location = GeoPoint(Angle::Degrees(lon), Angle::Degrees(lat));
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

static std::string_view
ExtractFirstTagValue(std::string_view text, std::string_view open_tag,
                     std::string_view close_tag) noexcept
{
  const std::size_t open = text.find(open_tag);
  if (open == std::string_view::npos)
    return {};

  const std::size_t value_begin = open + open_tag.size();
  const std::size_t close = text.find(close_tag, value_begin);
  if (close == std::string_view::npos)
    return {};

  return text.substr(value_begin, close - value_begin);
}

static std::string
NormalizeSelector(const std::string &selector)
{
  std::string s = std::string(Trim(selector));
  if (StringEndsWithIgnoreCase(s.c_str(), " points"))
    s.resize(s.size() - 7);

  return s;
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

static std::string
ExtractSourceKey(std::string_view body, std::string_view fallback_name)
{
  const std::string_view description = ExtractFirstTagValue(body, "<description>", "</description>");
  const std::string_view marker = "<b>Source:</b>";
  const std::size_t marker_pos = description.find(marker);
  if (marker_pos != std::string_view::npos)
  {
    std::string_view src = description.substr(marker_pos + marker.size());
    const std::size_t br = src.find("<br");
    if (br != std::string_view::npos)
      src = src.substr(0, br);

    src = Trim(src);
    if (!src.empty())
      return std::string(src);
  }

  std::string name = std::string(Trim(fallback_name));
  if (name.empty())
    return "Unnamed";

  const std::size_t space = name.find(' ');
  if (space != std::string::npos)
    name = name.substr(0, space);

  return name;
}

static std::map<std::string, std::vector<DebugReplayKML::Fix>>
ParsePointTimelines(const std::string &kml_text)
{
  std::map<std::string, std::vector<DebugReplayKML::Fix>> by_name;

  std::size_t pos = 0;
  while (true)
  {
    const std::size_t pm_start = kml_text.find("<Placemark", pos);
    if (pm_start == std::string::npos)
      break;

    const std::size_t pm_open_end = kml_text.find('>', pm_start);
    if (pm_open_end == std::string::npos)
      break;

    const std::size_t pm_end = kml_text.find("</Placemark>", pm_open_end + 1);
    if (pm_end == std::string::npos)
      break;

    const std::string_view body(kml_text.data() + pm_open_end + 1,
                                pm_end - pm_open_end - 1);

    const std::string_view when = ExtractFirstTagValue(body, "<when>", "</when>");
    const std::string_view coords = ExtractFirstTagValue(body, "<coordinates>", "</coordinates>");
    const std::string_view placemark_name = Trim(ExtractFirstTagValue(body, "<name>", "</name>"));

    if (!when.empty() && !coords.empty())
    {
      DebugReplayKML::Fix fix;
      if (ParseKmlWhenUTC(when, fix.date_time_utc) &&
          ParsePointCoordinates(coords, fix.location, fix.gps_altitude))
      {
        const std::string key = ExtractSourceKey(body, placemark_name);
        by_name[key].push_back(fix);
      }
    }

    pos = pm_end + 12;
  }

  for (auto it = by_name.begin(); it != by_name.end();)
  {
    auto &v = it->second;
    if (v.size() < 2)
    {
      it = by_name.erase(it);
      continue;
    }

    std::sort(v.begin(), v.end(), [](const auto &a, const auto &b)
              { return a.date_time_utc < b.date_time_utc; });
    ++it;
  }

  return by_name;
}

static InputSpec
ParseInputSpec(Path input_file)
{
  const std::string spec(input_file.c_str());
  const std::size_t hash = spec.find('#');
  if (hash == std::string::npos)
    return {spec, ""};

  return {spec.substr(0, hash), spec.substr(hash + 1)};
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

static std::string
ReadKmlOrKmzText(Path input_file)
{
  return input_file.EndsWithIgnoreCase(".kmz")
             ? ReadTextFromKmz(input_file)
             : ReadTextFile(input_file);
}

DebugReplayKML::DebugReplayKML(std::vector<Fix> &&_fixes) noexcept
  : fixes(std::move(_fixes))
{
}

std::vector<std::string>
DebugReplayKML::ListPointTimelineSources(Path input_file)
{
  const InputSpec spec = ParseInputSpec(input_file);
  const std::string content = ReadKmlOrKmzText(Path(spec.path.c_str()));

  const auto by_name = ParsePointTimelines(content);
  std::vector<std::string> names;
  names.reserve(by_name.size());
  for (const auto &kv : by_name)
    names.push_back(kv.first);

  return names;
}

DebugReplay *
DebugReplayKML::Create(Path input_file)
{
  try
  {
    const InputSpec spec = ParseInputSpec(input_file);
    const std::string content = ReadKmlOrKmzText(Path(spec.path.c_str()));

    if (spec.selector.empty())
    {
      try
      {
        auto fixes = ParseKmlGxTrack(content);
        return new DebugReplayKML(std::move(fixes));
      }
      catch (const std::exception &)
      {
      }

      auto by_name = ParsePointTimelines(content);
      if (by_name.empty())
        throw std::runtime_error("no supported timeline found (expected gx:Track or TimeStamp+Point placemarks)");

      if (by_name.size() > 1)
      {
        std::ostringstream oss;
        bool first = true;
        for (const auto &kv : by_name)
        {
          if (!first)
            oss << ", ";
          first = false;
          oss << kv.first;
        }

        throw std::runtime_error("multiple timeline sources found; specify selector as <file>#<source> (sources: " + oss.str() + ")");
      }

      return new DebugReplayKML(std::move(by_name.begin()->second));
    }

    const std::string selector = NormalizeSelector(spec.selector);

    auto by_name = ParsePointTimelines(content);
    auto it = by_name.find(selector);
    if (it != by_name.end())
      return new DebugReplayKML(std::move(it->second));

    throw std::runtime_error("selector not found in KML/KMZ point timelines: " + selector);
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
  return std::string("KML/KMZ timeline");
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
