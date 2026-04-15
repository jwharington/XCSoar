// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Geo/GeoPoint.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace MultiAircraft
{

    class SRTMTerrain
    {
        struct TileKey
        {
            int south;
            int west;

            bool operator==(const TileKey &) const noexcept = default;
        };

        struct Tile
        {
            int south;
            int west;
            unsigned dimension;
            std::vector<std::int16_t> samples;
        };

        struct TileKeyHash
        {
            std::size_t operator()(const TileKey &key) const noexcept;
        };

        std::unordered_map<TileKey, Tile, TileKeyHash> tiles;

        static Tile LoadTile(const std::string &path);
        static TileKey ParseTileKey(const std::string &path);
        const Tile *FindTile(double latitude, double longitude) const noexcept;

    public:
        SRTMTerrain() = default;
        explicit SRTMTerrain(const std::vector<std::string> &paths);

        void Load(const std::vector<std::string> &paths);

        [[nodiscard]] bool empty() const noexcept
        {
            return tiles.empty();
        }

        [[nodiscard]] std::optional<double> GetHeight(const GeoPoint &location) const noexcept;
        [[nodiscard]] std::optional<double> GetHeight(double latitude,
                                                      double longitude) const noexcept;

        struct Nearest3DResult
        {
            double distance_3d;
            double terrain_latitude;
            double terrain_longitude;
            double terrain_altitude;
        };

        [[nodiscard]] std::optional<Nearest3DResult> GetNearest3D(
            double latitude, double longitude, double altitude,
            double search_radius_m) const noexcept;
    };

}