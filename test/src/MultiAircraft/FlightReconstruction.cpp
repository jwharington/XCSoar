#include "FlightReconstruction.hpp"
#include "FlightReconstructionInternal.hpp"
#include "FlightReconstructionOptions.hpp"

namespace FlightReconstruction
{
  namespace
  {
    detail::NamedDefaults9 PROCESS_COVARIANCE_DEFAULTS{{
        {"x", 8.129376762766714},
        {"y", 8.129376762766714},
        {"z", 8.402678735062864},
        {"u", 1.112799331346204},
        {"w", 0.2877732710606822},
        {"q", 0.0002},
        {"attitude_x", 0.003095822118628868},
        {"attitude_y", 0.003095822118628868},
        {"attitude_z", 0.003095822118628868},
    }};

    detail::NamedDefaults10 PROCESS_COVARIANCE_DEFAULTS_WITH_UPDRAFT_GUST{{
        {"x", 10.918833735156165},
        {"y", 10.918833735156165},
        {"z", 19.525294936161085},
        {"u", 1.4989083887577612},
        {"w", 0.4711712877331108},
        {"q", 0.0002},
        {"w_g", 0.004230328130467635},
        {"attitude_x", 0.0042056840495608505},
        {"attitude_y", 0.0042056840495608505},
        {"attitude_z", 0.0042056840495608505},
    }};

    detail::MeasurementDefaults MEASUREMENT_COVARIANCE_DEFAULTS{{
        {"x", 26.907857595261536},
        {"y", 26.907857595261536},
        {"z", 26.57975010971075},
        {"v_tas", 4.942145400204616},
    }};

    detail::NamedDefaults9 STATE_COVARIANCE_DEFAULTS{{
        {"x", 194.46028610717067},
        {"y", 194.46028610717067},
        {"z", 1.2715498533675316},
        {"u", 0.010950140327450394},
        {"w", 0.07711954113735603},
        {"q", 0.0001215006964818211},
        {"attitude_x", 0.0012151160922442286},
        {"attitude_y", 0.0012151160922442286},
        {"attitude_z", 0.0012151160922442286},
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

    // Robust default from joint sigma + covariance tuning.
    double UKF_ALPHA = 0.3;
    double UKF_BETA = 4.0;
    double UKF_KAPPA = -3.0;
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

  void SetUKFWeightCoefficients(const double alpha,
                                const double beta,
                                const double kappa)
  {
    UKF_ALPHA = alpha;
    UKF_BETA = beta;
    UKF_KAPPA = kappa;
  }

  std::array<double, 3> GetUKFWeightCoefficients()
  {
    return {UKF_ALPHA, UKF_BETA, UKF_KAPPA};
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
