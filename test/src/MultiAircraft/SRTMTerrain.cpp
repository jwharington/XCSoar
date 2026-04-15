// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SRTMTerrain.hpp"
#include "io/FileReader.hxx"
#include "system/Path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace MultiAircraft
{

    namespace
    {
        constexpr std::int16_t SRTM_VOID = -32768;

        [[nodiscard]] static std::string
        Basename(const std::string &path)
        {
            const std::size_t pos = path.find_last_of("/\\");
            return pos == std::string::npos ? path : path.substr(pos + 1);
        }

        [[nodiscard]] static bool
        HasHgtExtension(const std::string &name) noexcept
        {
            return name.size() == 11 &&
                   (name[7] == '.') &&
                   (name[8] == 'h' || name[8] == 'H') &&
                   (name[9] == 'g' || name[9] == 'G') &&
                   (name[10] == 't' || name[10] == 'T');
        }

        [[nodiscard]] static int
        ParseUnsignedDigits(const std::string &s, std::size_t offset, std::size_t length)
        {
            int value = 0;
            for (std::size_t i = 0; i < length; ++i)
            {
                const char ch = s[offset + i];
                if (ch < '0' || ch > '9')
                    throw std::invalid_argument{"invalid SRTM tile filename: " + s};

                value = value * 10 + (ch - '0');
            }

            return value;
        }

        [[nodiscard]] static std::optional<double>
        BilinearInterpolate(std::int16_t h00, std::int16_t h10,
                            std::int16_t h01, std::int16_t h11,
                            double tx, double ty) noexcept
        {
            if (h00 == SRTM_VOID || h10 == SRTM_VOID ||
                h01 == SRTM_VOID || h11 == SRTM_VOID)
                return std::nullopt;

            const double north = h00 + (h10 - h00) * tx;
            const double south = h01 + (h11 - h01) * tx;
            return north + (south - north) * ty;
        }
    }

    std::size_t
    SRTMTerrain::TileKeyHash::operator()(const TileKey &key) const noexcept
    {
        return (std::size_t(unsigned(key.south + 256)) << 16) ^
               std::size_t(unsigned(key.west + 512));
    }

    SRTMTerrain::SRTMTerrain(const std::vector<std::string> &paths)
    {
        Load(paths);
    }

    void
    SRTMTerrain::Load(const std::vector<std::string> &paths)
    {
        if (paths.empty())
            throw std::invalid_argument{"terrain.files must not be empty"};

        tiles.clear();
        for (const auto &path : paths)
        {
            Tile tile = LoadTile(path);
            const TileKey key{tile.south, tile.west};
            const auto [it, inserted] = tiles.emplace(key, std::move(tile));
            if (!inserted)
                throw std::invalid_argument{"duplicate SRTM tile: " + path};
        }
    }

    SRTMTerrain::TileKey
    SRTMTerrain::ParseTileKey(const std::string &path)
    {
        const std::string name = Basename(path);
        if (!HasHgtExtension(name))
            throw std::invalid_argument{"unsupported SRTM tile filename: " + name};

        const char lat_hemisphere = name[0];
        const char lon_hemisphere = name[3];

        const int latitude = ParseUnsignedDigits(name, 1, 2);
        const int longitude = ParseUnsignedDigits(name, 4, 3);

        if (lat_hemisphere != 'N' && lat_hemisphere != 'S')
            throw std::invalid_argument{"invalid SRTM latitude hemisphere: " + name};

        if (lon_hemisphere != 'E' && lon_hemisphere != 'W')
            throw std::invalid_argument{"invalid SRTM longitude hemisphere: " + name};

        return {
            lat_hemisphere == 'S' ? -latitude : latitude,
            lon_hemisphere == 'W' ? -longitude : longitude,
        };
    }

    SRTMTerrain::Tile
    SRTMTerrain::LoadTile(const std::string &path)
    {
        const TileKey key = ParseTileKey(path);

        FileReader stream(Path(path.c_str()));
        const uint_least64_t size_u64 = stream.GetSize();
        if (size_u64 == 0 || (size_u64 % 2) != 0 ||
            size_u64 > std::numeric_limits<std::size_t>::max())
            throw std::runtime_error{"invalid SRTM tile size: " + path};

        const std::size_t size = std::size_t(size_u64);

        const std::size_t sample_count = std::size_t(size / 2);
        const auto dimension = unsigned(std::lround(std::sqrt(double(sample_count))));
        if (dimension < 2 || std::size_t(dimension) * dimension != sample_count)
            throw std::runtime_error{"SRTM tile is not square: " + path};

        std::vector<unsigned char> raw(size, 0);
        auto bytes = std::as_writable_bytes(std::span{raw});
        std::size_t n_read = 0;
        while (n_read < bytes.size())
        {
            const std::size_t n = stream.Read(bytes.subspan(n_read));
            if (n == 0)
                break;

            n_read += n;
        }

        if (n_read != bytes.size())
            throw std::runtime_error{"failed to read SRTM tile: " + path};

        std::vector<std::int16_t> samples(sample_count);
        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const std::size_t offset = i * 2;
            const std::uint16_t value = (std::uint16_t(raw[offset]) << 8) |
                                        std::uint16_t(raw[offset + 1]);
            samples[i] = std::int16_t(value);
        }

        return {key.south, key.west, dimension, std::move(samples)};
    }

    const SRTMTerrain::Tile *
    SRTMTerrain::FindTile(double latitude, double longitude) const noexcept
    {
        const TileKey key{int(std::floor(latitude)), int(std::floor(longitude))};
        const auto it = tiles.find(key);
        return it != tiles.end() ? &it->second : nullptr;
    }

    std::optional<double>
    SRTMTerrain::GetHeight(const GeoPoint &location) const noexcept
    {
        return GetHeight(location.latitude.Degrees(), location.longitude.Degrees());
    }

    std::optional<double>
    SRTMTerrain::GetHeight(double latitude, double longitude) const noexcept
    {
        const Tile *tile = FindTile(latitude, longitude);
        if (tile == nullptr)
            return std::nullopt;

        const double local_lat = std::clamp(latitude - tile->south, 0.0, 1.0);
        const double local_lon = std::clamp(longitude - tile->west, 0.0, 1.0);

        const double max_index = tile->dimension - 1;
        const double x = local_lon * max_index;
        const double y = (1.0 - local_lat) * max_index;

        const unsigned x0 = unsigned(std::floor(x));
        const unsigned y0 = unsigned(std::floor(y));
        const unsigned x1 = std::min(x0 + 1, tile->dimension - 1);
        const unsigned y1 = std::min(y0 + 1, tile->dimension - 1);

        const auto sample_at = [&](unsigned row, unsigned column)
        {
            return tile->samples[std::size_t(row) * tile->dimension + column];
        };

        return BilinearInterpolate(sample_at(y0, x0), sample_at(y0, x1),
                                   sample_at(y1, x0), sample_at(y1, x1),
                                   x - x0, y - y0);
    }

    std::optional<SRTMTerrain::Nearest3DResult>
    SRTMTerrain::GetNearest3D(double latitude, double longitude,
                              double altitude,
                              double search_radius_m) const noexcept
    {
        constexpr double DEG_TO_M_LAT = 111320.0;
        const double deg_to_m_lon = DEG_TO_M_LAT * std::cos(latitude * M_PI / 180.0);

        const double dlat = search_radius_m / DEG_TO_M_LAT;
        const double dlon = (deg_to_m_lon > 1.0)
                                ? search_radius_m / deg_to_m_lon
                                : search_radius_m / DEG_TO_M_LAT;

        // Determine the finest grid step from tiles covering the search area
        double step_lat = dlat;
        double step_lon = dlon;
        for (const auto &[key, tile] : tiles)
        {
            const double tile_step = 1.0 / (tile.dimension - 1);
            step_lat = std::min(step_lat, tile_step);
            step_lon = std::min(step_lon, tile_step);
            break; // all tiles share the same dimension
        }

        double best_dist_sq = std::numeric_limits<double>::max();
        double best_lat = latitude;
        double best_lon = longitude;
        double best_alt = altitude;
        bool found = false;

        for (double lat = latitude - dlat; lat <= latitude + dlat; lat += step_lat)
        {
            for (double lon = longitude - dlon; lon <= longitude + dlon; lon += step_lon)
            {
                const auto h = GetHeight(lat, lon);
                if (!h.has_value())
                    continue;

                const double dx = (lon - longitude) * deg_to_m_lon;
                const double dy = (lat - latitude) * DEG_TO_M_LAT;
                const double dz = *h - altitude;
                const double dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq < best_dist_sq)
                {
                    best_dist_sq = dist_sq;
                    best_lat = lat;
                    best_lon = lon;
                    best_alt = *h;
                    found = true;
                }
            }
        }

        if (!found)
            return std::nullopt;

        return Nearest3DResult{
            std::sqrt(best_dist_sq),
            best_lat,
            best_lon,
            best_alt};
    }

}