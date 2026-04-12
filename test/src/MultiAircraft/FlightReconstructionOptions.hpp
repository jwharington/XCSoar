#pragma once

#include <array>
#include <string_view>
#include <utility>

namespace FlightReconstruction
{
    void SetRTSWindowSize(unsigned value);
    bool SetProcessCovarianceDefault(std::string_view name, double value);
    bool SetProcessCovarianceDefaultWithUpdraftGust(std::string_view name, double value);
    bool SetMeasurementCovarianceDefault(std::string_view name, double value);
    bool SetStateCovarianceDefault(std::string_view name, double value);
    bool SetStateCovarianceDefaultWithUpdraftGust(std::string_view name, double value);

    std::array<std::pair<std::string_view, double>, 9> GetProcessCovarianceDefaults();
    std::array<std::pair<std::string_view, double>, 10> GetProcessCovarianceDefaultsWithUpdraftGust();
    std::array<std::pair<std::string_view, double>, 4> GetMeasurementCovarianceDefaults();
    std::array<std::pair<std::string_view, double>, 9> GetStateCovarianceDefaults();
    std::array<std::pair<std::string_view, double>, 10> GetStateCovarianceDefaultsWithUpdraftGust();
}
