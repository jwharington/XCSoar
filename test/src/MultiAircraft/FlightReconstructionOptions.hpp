#pragma once

#include <string_view>

namespace FlightReconstruction
{
    void SetRTSWindowSize(unsigned value);
    bool SetProcessCovarianceDefault(std::string_view name, double value);
    bool SetMeasurementCovarianceDefault(std::string_view name, double value);
    bool SetStateCovarianceDefault(std::string_view name, double value);
}
