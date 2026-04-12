// SPDX-License-Identifier: GPL-2.0-or-later

#include "MultiAircraft/FlightReconstruction.hpp"
#include "MultiAircraft/FlightReconstructionOptions.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    struct Sample
    {
        double time;
        double true_w_g;
        double estimated_w_g;
        double z;
    };

    double GustProfile(const double time)
    {
        if (time < 20.0)
            return 0.0;
        if (time < 55.0)
            return 2.0;
        if (time < 85.0)
            return -1.4;
        if (time < 115.0)
            return 0.9;
        return 0.0;
    }

    FlightReconstruction::Measurement MakeMeasurement(
        const FlightReconstruction::StateWithUpdraftGust &truth_state,
        const double time)
    {
        const auto &[states, attitude] = truth_state.data;
        (void)attitude;

        const double x_noise = 0.35 * std::sin(0.17 * time);
        const double y_noise = 0.15 * std::cos(0.11 * time);
        const double z_noise = 0.20 * std::sin(0.09 * time + 0.7);
        const double tas_noise = 0.05 * std::cos(0.13 * time + 0.2);

        return {
            states[FlightReconstruction::POS_X] + x_noise,
            states[FlightReconstruction::POS_Y] + y_noise,
            states[FlightReconstruction::POS_Z] + z_noise,
            std::hypot(states[FlightReconstruction::VEL_U],
                       states[FlightReconstruction::VEL_W]) +
                tas_noise,
        };
    }

    double GetEstimatedGust(const FlightReconstruction::StateWithUpdraftGust &state)
    {
        const auto &[states, attitude] = state.data;
        (void)attitude;
        return states[6];
    }

    std::string BuildPolyline(const std::vector<Sample> &samples,
                              const double x_min,
                              const double x_max,
                              const double y_min,
                              const double y_max,
                              const double width,
                              const double height,
                              const bool use_estimate)
    {
        const double x_span = std::max(1.0, x_max - x_min);
        const double y_span = std::max(1.0, y_max - y_min);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        for (const auto &sample : samples)
        {
            const double x = (sample.time - x_min) / x_span * width;
            const double value = use_estimate ? sample.estimated_w_g : sample.true_w_g;
            const double y = height - (value - y_min) / y_span * height;
            oss << x << ',' << y << ' ';
        }

        return oss.str();
    }

    void WriteCsv(const std::string &path, const std::vector<Sample> &samples)
    {
        std::ofstream file(path);
        file << "time_s,true_w_g_mps,estimated_w_g_mps,z_m\n";
        file << std::fixed << std::setprecision(6);
        for (const auto &sample : samples)
        {
            file << sample.time << ','
                 << sample.true_w_g << ','
                 << sample.estimated_w_g << ','
                 << sample.z << '\n';
        }
    }

    void WriteHtml(const std::string &path, const std::vector<Sample> &samples)
    {
        constexpr double width = 880.0;
        constexpr double height = 420.0;
        constexpr double margin_left = 72.0;
        constexpr double margin_right = 24.0;
        constexpr double margin_top = 34.0;
        constexpr double margin_bottom = 54.0;

        const double x_min = samples.front().time;
        const double x_max = samples.back().time;

        double y_min = samples.front().true_w_g;
        double y_max = samples.front().true_w_g;
        for (const auto &sample : samples)
        {
            y_min = std::min(y_min, std::min(sample.true_w_g, sample.estimated_w_g));
            y_max = std::max(y_max, std::max(sample.true_w_g, sample.estimated_w_g));
        }

        y_min -= 0.5;
        y_max += 0.5;

        const std::string true_polyline = BuildPolyline(samples, x_min, x_max, y_min, y_max,
                                                        width, height, false);
        const std::string estimated_polyline = BuildPolyline(samples, x_min, x_max, y_min, y_max,
                                                             width, height, true);

        std::ofstream file(path);
        file << "<!doctype html>\n"
             << "<html lang=\"en\">\n"
             << "<head>\n"
             << "  <meta charset=\"utf-8\">\n"
             << "  <title>Gust Filter Demo</title>\n"
             << "  <style>\n"
             << "    :root { color-scheme: light; }\n"
             << "    body { margin: 0; font-family: 'Iosevka Aile', 'IBM Plex Sans', sans-serif; background: linear-gradient(180deg, #f5f1e8 0%, #e9eef5 100%); color: #17212b; }\n"
             << "    main { max-width: 1040px; margin: 32px auto; padding: 28px; background: rgba(255,255,255,0.78); border: 1px solid rgba(23,33,43,0.12); box-shadow: 0 20px 60px rgba(23,33,43,0.12); backdrop-filter: blur(12px); }\n"
             << "    h1 { margin: 0 0 8px; font-size: 30px; letter-spacing: -0.03em; }\n"
             << "    p { margin: 0 0 20px; line-height: 1.5; }\n"
             << "    .legend { display: flex; gap: 18px; margin: 0 0 18px; font-size: 14px; }\n"
             << "    .legend span::before { content: ''; display: inline-block; width: 14px; height: 3px; margin-right: 8px; vertical-align: middle; }\n"
             << "    .legend .true::before { background: #14532d; }\n"
             << "    .legend .estimate::before { background: #b45309; }\n"
             << "    svg { width: 100%; height: auto; border-radius: 18px; background: #fffdfa; border: 1px solid rgba(23,33,43,0.12); }\n"
             << "    .axis { stroke: #485569; stroke-width: 1.2; }\n"
             << "    .grid { stroke: rgba(72,85,105,0.16); stroke-width: 1; }\n"
             << "    .true-line { fill: none; stroke: #14532d; stroke-width: 3; }\n"
             << "    .estimate-line { fill: none; stroke: #b45309; stroke-width: 3; stroke-dasharray: 10 7; }\n"
             << "    .label { fill: #334155; font-size: 13px; }\n"
             << "  </style>\n"
             << "</head>\n"
             << "<body>\n"
             << "<main>\n"
             << "  <h1>Flight Reconstruction Gust Filter Demo</h1>\n"
             << "  <p>This synthetic example drives the gust-aware UKF with position and true-airspeed measurements only. The plot compares the injected updraft gust profile with the filter estimate of <code>w_g</code>.</p>\n"
             << "  <div class=\"legend\"><span class=\"true\">True w_g</span><span class=\"estimate\">Estimated w_g</span></div>\n"
             << "  <svg viewBox=\"0 0 976 508\" xmlns=\"http://www.w3.org/2000/svg\">\n";

        for (int i = 0; i <= 6; ++i)
        {
            const double y = margin_top + i * (height / 6.0);
            file << "    <line class=\"grid\" x1=\"" << margin_left << "\" y1=\"" << y
                 << "\" x2=\"" << (margin_left + width) << "\" y2=\"" << y << "\" />\n";
        }

        for (int i = 0; i <= 6; ++i)
        {
            const double x = margin_left + i * (width / 6.0);
            file << "    <line class=\"grid\" x1=\"" << x << "\" y1=\"" << margin_top
                 << "\" x2=\"" << x << "\" y2=\"" << (margin_top + height) << "\" />\n";
        }

        file << "    <g transform=\"translate(" << margin_left << ',' << margin_top << ")\">\n"
             << "      <polyline class=\"true-line\" points=\"" << true_polyline << "\" />\n"
             << "      <polyline class=\"estimate-line\" points=\"" << estimated_polyline << "\" />\n"
             << "    </g>\n"
             << "    <line class=\"axis\" x1=\"" << margin_left << "\" y1=\"" << (margin_top + height)
             << "\" x2=\"" << (margin_left + width) << "\" y2=\"" << (margin_top + height) << "\" />\n"
             << "    <line class=\"axis\" x1=\"" << margin_left << "\" y1=\"" << margin_top
             << "\" x2=\"" << margin_left << "\" y2=\"" << (margin_top + height) << "\" />\n";

        file << std::fixed << std::setprecision(1);
        for (int i = 0; i <= 6; ++i)
        {
            const double tick_time = x_min + i * (x_max - x_min) / 6.0;
            const double x = margin_left + i * (width / 6.0);
            file << "    <text class=\"label\" x=\"" << x << "\" y=\"" << (margin_top + height + 28.0)
                 << "\" text-anchor=\"middle\">" << tick_time << " s</text>\n";
        }

        for (int i = 0; i <= 6; ++i)
        {
            const double tick_value = y_max - i * (y_max - y_min) / 6.0;
            const double y = margin_top + i * (height / 6.0) + 4.0;
            file << "    <text class=\"label\" x=\"" << (margin_left - 12.0) << "\" y=\"" << y
                 << "\" text-anchor=\"end\">" << tick_value << "</text>\n";
        }

        file << "    <text class=\"label\" x=\"" << (margin_left + width / 2.0) << "\" y=\"488\" text-anchor=\"middle\">time (s)</text>\n"
             << "    <text class=\"label\" x=\"22\" y=\"" << (margin_top + height / 2.0)
             << "\" text-anchor=\"middle\" transform=\"rotate(-90 22 " << (margin_top + height / 2.0) << ")\">w_g (m/s)</text>\n"
             << "  </svg>\n"
             << "</main>\n"
             << "</body>\n"
             << "</html>\n";
    }
}

int main(int argc, char **argv)
{
    const std::string prefix = argc > 1 ? argv[1] : "gust_filter_demo";
    const std::string csv_path = prefix + ".csv";
    const std::string html_path = prefix + ".html";

    constexpr double dt = 1.0;
    constexpr unsigned steps = 140;

    FlightReconstruction::SetRTSWindowSize(20);
    FlightReconstruction::SetProcessCovarianceDefaultWithUpdraftGust("w_g", 0.18);
    FlightReconstruction::SetStateCovarianceDefaultWithUpdraftGust("w_g", 1.5);

    FlightReconstruction::FilterWithUpdraftGust truth_model;
    FlightReconstruction::FilterWithUpdraftGust estimator;

    auto truth_state = FlightReconstruction::get_initial_state_estimate_with_updraft_gust(
        0.0, 0.0, -1200.0,
        28.0, 0.0,
        0.0, 0.0,
        0.0);
    auto estimate_state = FlightReconstruction::get_initial_state_estimate_with_updraft_gust(
        0.0, 0.0, -1200.0,
        28.0, 0.0,
        0.0, 0.0,
        0.0);
    estimator.initialise(estimate_state, dt);

    std::vector<Sample> samples;
    samples.reserve(steps);

    for (unsigned step = 0; step < steps; ++step)
    {
        const double time = (step + 1) * dt;
        auto &[truth_linear, truth_attitude] = truth_state.data;
        (void)truth_attitude;
        truth_linear[6] = GustProfile(time);

        truth_model.system_model(truth_state, dt);
        const auto measurement = MakeMeasurement(truth_state, time);
        estimator.update(measurement, dt);

        const auto &estimated_state = estimator.get_state();
        const auto &[truth_states, ignored_attitude] = truth_state.data;
        (void)ignored_attitude;

        samples.push_back({
            time,
            truth_states[6],
            GetEstimatedGust(estimated_state),
            truth_states[FlightReconstruction::POS_Z],
        });
    }

    WriteCsv(csv_path, samples);
    WriteHtml(html_path, samples);

    std::cout << "Wrote gust filter demo outputs:\n"
              << "  CSV:  " << csv_path << '\n'
              << "  HTML: " << html_path << '\n';
    return 0;
}