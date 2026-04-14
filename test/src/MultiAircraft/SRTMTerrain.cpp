// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SRTMTerrain.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
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

        std::ifstream stream(path, std::ios::binary | std::ios::ate);
        if (!stream.is_open())
            throw std::runtime_error{"failed to open SRTM tile: " + path};

        const std::streamsize size = stream.tellg();
        if (size <= 0 || (size % 2) != 0)
            throw std::runtime_error{"invalid SRTM tile size: " + path};

        stream.seekg(0, std::ios::beg);

        const std::size_t sample_count = std::size_t(size / 2);
        const auto dimension = unsigned(std::lround(std::sqrt(double(sample_count))));
        if (dimension < 2 || std::size_t(dimension) * dimension != sample_count)
            throw std::runtime_error{"SRTM tile is not square: " + path};

        std::vector<unsigned char> raw(std::size_t(size), 0);
        if (!stream.read(reinterpret_cast<char *>(raw.data()), size))
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

}