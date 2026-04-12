#include "FlightReconstruction.hpp"
#include "FlightReconstructionInternal.hpp"
#include "FlightReconstructionOptions.hpp"

namespace FlightReconstruction
{
  namespace
  {
    detail::NamedDefaults9 PROCESS_COVARIANCE_DEFAULTS{{
        {"x", 10.052597062015902},
        {"y", 10.052597062015902},
        {"z", 10.975253850744147},
        {"u", 1.5527694552714049},
        {"w", 0.46489285919931596},
        {"q", 0.00016329692067816702},
        {"attitude_x", 0.0048374875779941988},
        {"attitude_y", 0.0048374875779941988},
        {"attitude_z", 0.0048374875779941988},
    }};

    detail::NamedDefaults10 PROCESS_COVARIANCE_DEFAULTS_WITH_UPDRAFT_GUST{{
        {"x", 10.918833735156165},
        {"y", 10.918833735156165},
        {"z", 19.525294936161085},
        {"u", 1.4989083887577612},
        {"w", 0.4711712877331108},
        {"q", 0.0001},
        {"w_g", 0.004230328130467635},
        {"attitude_x", 0.0042056840495608505},
        {"attitude_y", 0.0042056840495608505},
        {"attitude_z", 0.0042056840495608505},
    }};

    detail::MeasurementDefaults MEASUREMENT_COVARIANCE_DEFAULTS{{
        {"x", 37.83580017991135},
        {"y", 37.83580017991135},
        {"z", 38.67570798631411},
        {"v_tas", 7.141561460562895},
    }};

    detail::NamedDefaults9 STATE_COVARIANCE_DEFAULTS{{
        {"x", 380.81186001478410},
        {"y", 380.81186001478410},
        {"z", 2.3803373717706506},
        {"u", 0.019631490095117325},
        {"w", 0.12451191238925761},
        {"q", 0.00024300000000000024},
        {"attitude_x", 0.0024300000000000016},
        {"attitude_y", 0.0024300000000000016},
        {"attitude_z", 0.0024300000000000016},
    }};

    detail::NamedDefaults10 STATE_COVARIANCE_DEFAULTS_WITH_UPDRAFT_GUST{{
        {"x", 243.43607204032173},
        {"y", 243.43607204032173},
        {"z", 2.732737093014798},
        {"u", 0.05064069790064298},
        {"w", 0.3991957029509824},
        {"q", 0.0001},
        {"w_g", 0.012000000000000002},
        {"attitude_x", 0.0007290000000000006},
        {"attitude_y", 0.0007290000000000006},
        {"attitude_z", 0.0007290000000000006},
    }};
  }

  detail::NamedDefaults9 &detail::ProcessCovarianceDefaultsStorage()
  {
    return PROCESS_COVARIANCE_DEFAULTS;
  }

  detail::NamedDefaults10 &detail::ProcessCovarianceDefaultsWithUpdraftGustStorage()
  {
    return PROCESS_COVARIANCE_DEFAULTS_WITH_UPDRAFT_GUST;
  }

  detail::MeasurementDefaults &detail::MeasurementCovarianceDefaultsStorage()
  {
    return MEASUREMENT_COVARIANCE_DEFAULTS;
  }

  detail::NamedDefaults9 &detail::StateCovarianceDefaultsStorage()
  {
    return STATE_COVARIANCE_DEFAULTS;
  }

  detail::NamedDefaults10 &detail::StateCovarianceDefaultsWithUpdraftGustStorage()
  {
    return STATE_COVARIANCE_DEFAULTS_WITH_UPDRAFT_GUST;
  }

  unsigned Filter::RTS_WINDOW_SIZE = 20;
  unsigned FilterWithUpdraftGust::RTS_WINDOW_SIZE = 20;

  void SetRTSWindowSize(unsigned value)
  {
    Filter::RTS_WINDOW_SIZE = value;
    FilterWithUpdraftGust::RTS_WINDOW_SIZE = value;
  }

  bool SetProcessCovarianceDefault(const std::string_view name,
                                   const double value)
  {
    return SetNamedDefault(detail::ProcessCovarianceDefaultsStorage(), name, value);
  }

  bool SetMeasurementCovarianceDefault(const std::string_view name,
                                       const double value)
  {
    return SetNamedDefault(detail::MeasurementCovarianceDefaultsStorage(), name, value);
  }

  bool SetProcessCovarianceDefaultWithUpdraftGust(const std::string_view name,
                                                  const double value)
  {
    return SetNamedDefault(detail::ProcessCovarianceDefaultsWithUpdraftGustStorage(), name, value);
  }

  bool SetStateCovarianceDefault(const std::string_view name,
                                 const double value)
  {
    return SetNamedDefault(detail::StateCovarianceDefaultsStorage(), name, value);
  }

  bool SetStateCovarianceDefaultWithUpdraftGust(const std::string_view name,
                                                const double value)
  {
    return SetNamedDefault(detail::StateCovarianceDefaultsWithUpdraftGustStorage(), name, value);
  }

  std::array<std::pair<std::string_view, double>, 9> GetProcessCovarianceDefaults()
  {
    return detail::ProcessCovarianceDefaultsStorage();
  }

  std::array<std::pair<std::string_view, double>, 4> GetMeasurementCovarianceDefaults()
  {
    return detail::MeasurementCovarianceDefaultsStorage();
  }

  std::array<std::pair<std::string_view, double>, 9> GetStateCovarianceDefaults()
  {
    return detail::StateCovarianceDefaultsStorage();
  }

  std::array<std::pair<std::string_view, double>, 10> GetProcessCovarianceDefaultsWithUpdraftGust()
  {
    return detail::ProcessCovarianceDefaultsWithUpdraftGustStorage();
  }

  std::array<std::pair<std::string_view, double>, 10> GetStateCovarianceDefaultsWithUpdraftGust()
  {
    return detail::StateCovarianceDefaultsWithUpdraftGustStorage();
  }

};
