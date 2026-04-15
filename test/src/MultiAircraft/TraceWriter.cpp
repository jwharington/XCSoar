// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TraceWriter.hpp"
#include "AircraftModel.hpp"
#include "DetectMiss.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "FlightReconstruction.hpp"
#include "ReconstructionUtility.hpp"
#include <type_traits>

namespace
{
    using namespace MultiAircraft;
    constexpr bool uses_updraft_gust_filter(const int filter_type)
    {
        return filter_type == 3 || filter_type == 4;
    }

    constexpr bool uses_smoothed_position(const int filter_type)
    {
        return filter_type == 2 || filter_type == 4;
    }

    void update_visibility_avg(Averager &visibility_avg, const Visibility &visibility)
    {
        visibility_avg.add(visibility.focus_factor * (1 - visibility.occlusion));
    }

    void append_visibility_fields(boost::json::object &step,
                                  const Aspect &aspect,
                                  const Visibility &visibility)
    {
        step.emplace("range", aspect.range);
        step.emplace("elevation_angle", aspect.elevation_angle.Degrees());
        step.emplace("azimuth_angle", aspect.azimuth_angle.Degrees());
        step.emplace("inclination_angle", aspect.inclination_angle.Degrees());
        step.emplace("ang_size", visibility.angular_size.Degrees());
        step.emplace("occlusion", visibility.occlusion);
        step.emplace("focus_factor", visibility.focus_factor);
    }

    inline double get_gps_altitude(const TrailPoint &p) { return p.pos.gps_altitude; }
    inline double get_gps_altitude(const EventTrailSample &p) { return p.gps_altitude; }

    template <typename PointType>
    void append_raw_attitude_fields(boost::json::object &step, const PointType &p)
    {
        step.emplace("bank", p.bank_angle.Degrees());
        step.emplace("pitch", p.pitch_angle.Degrees());
        step.emplace("yaw", p.yaw_angle.AsBearing().Degrees());
    }

    template <typename PointType>
    void append_raw_flight_fields(boost::json::object &step, const PointType &p)
    {
        step.emplace("v_tas", p.v_tas);
        step.emplace("v_ias", p.v_ias);
        append_raw_attitude_fields(step, p);
        step.emplace("load_factor", p.load_factor);
    }

    template <typename PointType>
    void append_position_fields(boost::json::object &step, const FlatPoint &fp,
                                const PointType &p)
    {
        step.emplace("x", fp.x);
        step.emplace("y", fp.y);
        step.emplace("alt_gps", get_gps_altitude(p));
    }

    void append_common_detailed_fields(boost::json::object &step,
                                       const TrailPoint &p,
                                       const DetectMiss &miss)
    {
        step.emplace("turnrate", p.turn_rate_wind.Degrees());
        step.emplace("turn_mode", TurnModeList::to_string(p.turn_mode));
        step.emplace("actual", p.actual);
        step.emplace("plausible", p.plausible);
        step.emplace("fix_acc", p.fix_acc);
        step.emplace("roc", p.roc);
        step.emplace("miss_TCA", miss.TCA);
        step.emplace("d_mag", miss.d_mag);
        step.emplace("miss_d", miss.miss_d_mag);
        step.emplace("miss_vrel", miss.vrel_mag);
        step.emplace("distance_scale", miss.distance_scale);
    }

    void append_common_zem_fields(boost::json::object &step, const DetectMiss &miss,
                                  const FlatPoint &fp, const TrailPoint &p)
    {
        step.emplace("miss_x", fp.x + p.vel[0] * miss.TCA);
        step.emplace("miss_y", fp.y + p.vel[1] * miss.TCA);
        step.emplace("miss_z", p.pos.gps_altitude + p.vel[2] * miss.TCA);
    }

    template <typename FilterType, typename PointType>
    inline void initialise_filter_state(FilterType &filter,
                                        const FlatPoint &fp,
                                        const PointType &p)
    {
        if constexpr (std::is_same_v<FilterType, FlightReconstruction::FilterWithUpdraftGust>)
        {
            auto state = FlightReconstruction::get_initial_state_estimate_with_updraft_gust(
                fp.y, fp.x,
                -get_gps_altitude(p),
                p.v_tas,
                p.bank_angle.Radians(),
                p.pitch_angle.Radians(),
                p.yaw_angle.AsBearing().Radians());
            filter.initialise(state, 1.0);
        }
        else
        {
            auto state = FlightReconstruction::get_initial_state_estimate(fp.y, fp.x,
                                                                          -get_gps_altitude(p),
                                                                          p.v_tas,
                                                                          p.bank_angle.Radians(),
                                                                          p.pitch_angle.Radians(),
                                                                          p.yaw_angle.AsBearing().Radians());
            filter.initialise(state, 1.0);
        }
    }

    template <typename FilterType, typename PointType>
    bool update_encounter_filter(FilterType &filter,
                                 const bool initialise,
                                 const FlatPoint &fp,
                                 const PointType &p)
    {
        if (initialise)
        {
            initialise_filter_state(filter, fp, p);
        }

        FlightReconstruction::Measurement measurement;
        auto &[y, x, z, U] = measurement.data;
        x.value = fp.x;
        y.value = fp.y;
        z.value = -get_gps_altitude(p);
        U.value = p.v_tas;

        try
        {
            filter.update(measurement, 1.0);
            return true;
        }
        catch (const std::exception &e)
        {
            // std::cerr << "Filter update failed: " << e.what() << "\n";
            return false;
        }
    }

    template <typename StateType>
    FlightReconstruction::Euler get_state_euler(const StateType &state)
    {
        auto &[states, quaternion] = state.data;
        (void)states;
        const Eigen::Matrix3d R = quaternion.get_q().toRotationMatrix();
        double theta = asin(-R(2, 0));
        double psi = acos(R(0, 0) / cos(theta)) * FlightReconstruction::sign(R(1, 0));
        double phi = acos(R(2, 2) / cos(theta)) * FlightReconstruction::sign(R(2, 1));
        if (psi < 0)
            psi += 2 * M_PI;

        return FlightReconstruction::Euler(phi, theta, psi) / FlightReconstruction::DEGTORAD;
    }

    template <typename StateType>
    FlightReconstruction::DerivState get_state_vector(const StateType &state)
    {
        auto &[states, quaternion] = state.data;
        FlightReconstruction::DerivState dstate;
        dstate.reserve(states.size() + 4);
        for (Eigen::Index i = 0; i < states.size(); ++i)
            dstate.push_back(states[i]);

        const auto &q = quaternion.get_q();
        dstate.push_back(q.w());
        dstate.push_back(q.x());
        dstate.push_back(q.y());
        dstate.push_back(q.z());
        return dstate;
    }

    template <typename FilterType>
    void append_smoothed_fields(boost::json::array &trace,
                                const FilterType &filter,
                                const size_t trace_offset,
                                const int filter_type,
                                const bool detailed,
                                const std::vector<DetectMiss> &misses,
                                Averager &visibility_avg)
    {
        const auto &smoothed_states = filter.get_smoothed_states();
        if (trace_offset >= smoothed_states.size())
            return;

        const size_t n = std::min(trace.size(), smoothed_states.size() - trace_offset);

        for (size_t i = 0; i < n; ++i)
        {
            auto &_step = trace[i].as_object();
            const auto &state = smoothed_states[i + trace_offset];
            const auto seuler = get_state_euler(state);
            const auto &aero = filter.get_aero(state);
            const auto dstate = get_state_vector(state);
            _step["bank"] = seuler[0];
            _step["pitch"] = seuler[1];
            _step["yaw"] = seuler[2];
            _step["load_factor"] = aero.load_factor;
            _step["alpha"] = aero.alpha / FlightReconstruction::DEGTORAD;
            _step["v_ias"] = aero.V_ias;
            _step["v_tas"] = aero.V_tas;

            if (uses_updraft_gust_filter(filter_type) &&
                dstate.size() > FlightReconstruction::QUATERNION + 4)
            {
                _step["w_g"] = dstate[FlightReconstruction::QUATERNION];
            }

            if (uses_smoothed_position(filter_type))
            {
                _step["y"] = dstate[FlightReconstruction::POS_X];
                _step["x"] = dstate[FlightReconstruction::POS_Y];
                _step["alt_gps"] = -dstate[FlightReconstruction::POS_Z];
            }

            if (!detailed)
                continue;

            const EulerAngles euler(Angle::Degrees(seuler[0]),
                                    Angle::Degrees(seuler[1]),
                                    Angle::Degrees(seuler[2]));
            if (i >= misses.size())
                continue;

            const auto &miss = misses[i];
            const Aspect aspect = euler.get_aspect(miss.xrel);
            const Visibility visibility(aspect);

            append_visibility_fields(_step, aspect, visibility);
            if (_step.find("t") != _step.end() && _step.at("t").to_number<double>() <= 0)
                update_visibility_avg(visibility_avg, visibility);
        }
    }

    template <typename Fn>
    void dispatch_filter(const int filter_type, Fn &&fn)
    {
        if (uses_updraft_gust_filter(filter_type))
        {
            FlightReconstruction::FilterWithUpdraftGust filter;
            fn(filter);
        }
        else
        {
            FlightReconstruction::Filter filter;
            fn(filter);
        }
    }

    template <typename FilterType>
    void populate_encounter_trace(const AircraftModel &aircraft,
                                  const EncounterMapStore::EncounterInfo &info,
                                  const unsigned id_target,
                                  const bool detailed,
                                  FilterType &filter,
                                  boost::json::array &trace,
                                  bool &plausible,
                                  Averager &visibility_avg,
                                  std::vector<DetectMiss> &misses,
                                  bool &kf_valid,
                                  size_t &reconstruction_warmup_samples)
    {
        const TimeStamp t_min = info.time_start - FloatDuration(EncounterMapStore::TYP_TRAIL);
        const TimeStamp t_max = info.time_end + FloatDuration(EncounterMapStore::HYS_TRAIL);
        const TimeStamp t_reconstruction_min = t_min - AircraftModel::reconstruction_pre_buffer;
        const bool filter_enabled = AircraftModel::filter_type > 0;

        for (auto &&p : aircraft.GetTrail())
        {
            if (!p.within_time(t_reconstruction_min, t_max))
                continue;

            const FlatPoint fp = info.project_loc_wind(p);

            if (filter_enabled)
                kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

            if (p.pos.time < t_min)
            {
                if (filter_enabled)
                    ++reconstruction_warmup_samples;
                continue;
            }

            const AuxiliaryPair &auxiliary = p.get_auxiliary(id_target);
            const DetectMiss &miss = auxiliary.second;

            if (detailed)
            {
                plausible &= p.plausible;
                misses.push_back(miss);
            }
            const bool proc_visible = p.pos.time <= info.time_start;

            boost::json::object step = {
                {"t", (p.pos.time - info.time_start).count()},
                {"alt_baro", p.pos.baro_altitude},
                {"v", p.v_wind.norm},
                {"hdg", p.v_wind.bearing.Degrees()},
            };
            append_raw_attitude_fields(step, p);

            const bool need_raw_flight = !kf_valid;
            const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

            if (need_raw_flight)
                append_raw_flight_fields(step, p);
            if (need_raw_position)
                append_position_fields(step, fp, p);

            if (detailed)
            {
                const Aspect &aspect = auxiliary.first;
                const Visibility visibility(aspect);
                append_common_detailed_fields(step, p, miss);
                append_common_zem_fields(step, miss, fp, p);

                if (need_raw_flight)
                {
                    append_visibility_fields(step, aspect, visibility);
                    if (proc_visible)
                        update_visibility_avg(visibility_avg, visibility);
                }
            }

            trace.emplace_back(step);
        }
    }

    template <typename FilterType>
    void populate_vignette_trace(const AircraftModel &aircraft,
                                 const Vignette &info,
                                 FilterType &filter,
                                 boost::json::array &trace,
                                 bool &plausible,
                                 bool &kf_valid,
                                 size_t &reconstruction_warmup_samples)
    {
        const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
        const bool filter_enabled = AircraftModel::filter_type > 0;

        for (auto &&p : aircraft.GetTrail())
        {
            if (!p.within_time(t_reconstruction_min, info.time_end))
                continue;

            const FlatPoint fp = info.project_loc_wind(p);

            if (filter_enabled)
                kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

            if (p.pos.time < info.time_start)
            {
                if (filter_enabled)
                    ++reconstruction_warmup_samples;
                continue;
            }

            boost::json::object step = {
                {"t", (p.pos.time - info.time_start).count()},
                {"alt_baro", p.pos.baro_altitude},
                {"v", p.v_wind.norm},
                {"hdg", p.v_wind.bearing.Degrees()},
            };
            append_raw_attitude_fields(step, p);

            const bool need_raw_flight = !kf_valid;
            const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

            if (need_raw_flight)
                append_raw_flight_fields(step, p);
            if (need_raw_position)
                append_position_fields(step, fp, p);

            plausible &= p.plausible;
            trace.emplace_back(step);
        }
    }

    template <typename FilterType>
    void populate_incursion_trace(const Vignette &info,
                                  const std::vector<std::pair<TimeStamp, double>> &depth_samples,
                                  const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
                                  const std::vector<EventTrailSample> &trail_samples,
                                  FilterType &filter,
                                  boost::json::array &trace,
                                  bool &plausible,
                                  bool &kf_valid,
                                  size_t &reconstruction_warmup_samples)
    {
        const TimeStamp t_min = info.time_start - FloatDuration(EncounterMapStore::TYP_TRAIL);
        const TimeStamp t_max = info.time_end + FloatDuration(EncounterMapStore::HYS_TRAIL);
        const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
        const bool filter_enabled = AircraftModel::filter_type > 0;
        auto depth_it = depth_samples.begin();
        auto boundary_it = boundary_samples.begin();
        double current_depth = 0;
        GeoPoint current_boundary_location = info.origin;
        double current_boundary_altitude = info.alt;

        for (const auto &p : trail_samples)
        {
            if (p.time < t_reconstruction_min || p.time > t_max)
                continue;

            const FlatPoint fp = info.project_loc_wind(p.location, p.time);

            if (filter_enabled)
                kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

            if (p.time < t_min)
            {
                if (filter_enabled)
                    ++reconstruction_warmup_samples;
                continue;
            }

            while (depth_it != depth_samples.end() && depth_it->first <= p.time)
            {
                current_depth = depth_it->second;
                ++depth_it;
            }

            while (boundary_it != boundary_samples.end() && std::get<0>(*boundary_it) <= p.time)
            {
                current_boundary_location = std::get<1>(*boundary_it);
                current_boundary_altitude = std::get<2>(*boundary_it);
                ++boundary_it;
            }

            const bool within_event = p.time >= info.time_start && p.time <= info.time_end;
            const bool has_positive_depth = within_event && current_depth > 0;

            boost::json::object incursion = {
                {"depth", within_event ? current_depth : 0.0},
            };
            if (has_positive_depth)
            {
                const FlatPoint boundary_fp = info.project_loc_wind(current_boundary_location, p.time);
                incursion.emplace("x", boundary_fp.x);
                incursion.emplace("y", boundary_fp.y);
                incursion.emplace("alt", current_boundary_altitude);
            }

            boost::json::object step = {
                {"t", (p.time - info.time_start).count()},
                {"alt_baro", p.baro_altitude},
                {"v", p.v_wind.norm},
                {"hdg", p.v_wind.bearing.Degrees()},
                {"incursion", std::move(incursion)},
            };
            append_raw_attitude_fields(step, p);

            const bool need_raw_flight = !kf_valid;
            const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

            if (need_raw_flight)
                append_raw_flight_fields(step, p);
            if (need_raw_position)
                append_position_fields(step, fp, p);

            plausible &= p.plausible;
            trace.emplace_back(step);
        }
    }

    template <typename FilterType>
    void populate_terrain_trace(const Vignette &info,
                                const std::vector<TerrainDistanceSample> &distance_samples,
                                const std::vector<EventTrailSample> &trail_samples,
                                FilterType &filter,
                                boost::json::array &trace,
                                bool &plausible,
                                bool &kf_valid,
                                size_t &reconstruction_warmup_samples)
    {
        const TimeStamp t_min = info.time_start - FloatDuration(EncounterMapStore::TYP_TRAIL);
        const TimeStamp t_max = info.time_end + FloatDuration(EncounterMapStore::HYS_TRAIL);
        const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
        const bool filter_enabled = AircraftModel::filter_type > 0;
        auto distance_it = distance_samples.begin();
        double current_distance = 0;
        double current_terrain_lat = 0;
        double current_terrain_lon = 0;
        double current_terrain_alt = 0;
        bool has_distance = false;

        for (const auto &p : trail_samples)
        {
            if (p.time < t_reconstruction_min || p.time > t_max)
                continue;

            const FlatPoint fp = info.project_loc_wind(p.location, p.time);

            if (filter_enabled)
                kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

            if (p.time < t_min)
            {
                if (filter_enabled)
                    ++reconstruction_warmup_samples;
                continue;
            }

            while (distance_it != distance_samples.end() && distance_it->time <= p.time)
            {
                current_distance = distance_it->distance;
                current_terrain_lat = distance_it->terrain_latitude;
                current_terrain_lon = distance_it->terrain_longitude;
                current_terrain_alt = distance_it->terrain_altitude;
                has_distance = true;
                ++distance_it;
            }

            boost::json::object terrain;
            if (has_distance)
                terrain["distance"] = current_distance;

            if (has_distance && current_distance < 0)
            {
                const GeoPoint terrain_loc{Angle::Degrees(current_terrain_lon),
                                           Angle::Degrees(current_terrain_lat)};
                const FlatPoint terrain_fp = info.project_loc_wind(terrain_loc, p.time);
                terrain["x"] = terrain_fp.x;
                terrain["y"] = terrain_fp.y;
                terrain["alt"] = current_terrain_alt;
            }

            boost::json::object step = {
                {"t", (p.time - info.time_start).count()},
                {"alt_baro", p.baro_altitude},
                {"v", p.v_wind.norm},
                {"hdg", p.v_wind.bearing.Degrees()},
                {"terrain", std::move(terrain)},
            };
            append_raw_attitude_fields(step, p);

            const bool need_raw_flight = !kf_valid;
            const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

            if (need_raw_flight)
                append_raw_flight_fields(step, p);
            if (need_raw_position)
                append_position_fields(step, fp, p);

            plausible &= p.plausible;
            trace.emplace_back(step);
        }
    }

} // anonymous namespace

namespace MultiAircraft::TraceWriter
{

    boost::json::object write_encounter(
        const AircraftModel &aircraft,
        const EncounterMapStore::EncounterInfo &info,
        const unsigned id_target,
        const bool detailed)
    {
        boost::json::array trace;

        bool plausible = true;
        Averager visibility_avg;
        std::vector<DetectMiss> misses;
        bool kf_valid = AircraftModel::filter_type > 0;
        size_t reconstruction_warmup_samples = 0;

        dispatch_filter(AircraftModel::filter_type, [&](auto &filter)
                        {
    populate_encounter_trace(aircraft, info, id_target, detailed, filter, trace,
                             plausible, visibility_avg, misses,
                             kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             AircraftModel::filter_type, detailed, misses, visibility_avg); });

        visibility_avg.calculate();

        char date_buffer[32];
        FormatISO8601(date_buffer, aircraft.flight_date_utc_start);

        boost::json::object data = {
            {"id", aircraft.id},
            {"fr_info", aircraft.fr_info},
            {"fr_id", aircraft.fr_id},
            {"turn_mode_list", aircraft.gen_turnmodelist(info).string()},
            {"in_flock", aircraft.in_flock},
            {"plausible", plausible},
            {"date_start", date_buffer},
            {"visibility_avg", visibility_avg.get_avg()},
            {"trace", trace}};

        return data;
    }

    boost::json::object write_vignette(
        const AircraftModel &aircraft,
        const Vignette &info)
    {
        boost::json::array trace;

        bool plausible = true;
        Averager visibility_avg;
        std::vector<DetectMiss> misses;
        bool kf_valid = AircraftModel::filter_type > 0;
        size_t reconstruction_warmup_samples = 0;

        dispatch_filter(AircraftModel::filter_type, [&](auto &filter)
                        {
    populate_vignette_trace(aircraft, info, filter, trace, plausible,
                            kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             AircraftModel::filter_type, false, misses, visibility_avg); });

        char date_buffer[32];
        FormatISO8601(date_buffer, aircraft.flight_date_utc_start);

        boost::json::object data = {
            {"id", aircraft.id},
            {"fr_info", aircraft.fr_info},
            {"fr_id", aircraft.fr_id},
            {"in_flock", aircraft.in_flock},
            {"plausible", plausible},
            {"date_start", date_buffer},
            {"trace", trace}};

        return data;
    }

    boost::json::object write_incursion(
        const AircraftModel &aircraft,
        const Vignette &info,
        const std::vector<std::pair<TimeStamp, double>> &depth_samples,
        const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
        const std::vector<EventTrailSample> &trail_samples)
    {
        boost::json::array trace;

        bool plausible = true;
        Averager visibility_avg;
        std::vector<DetectMiss> misses;
        bool kf_valid = AircraftModel::filter_type > 0;
        size_t reconstruction_warmup_samples = 0;

        dispatch_filter(AircraftModel::filter_type, [&](auto &filter)
                        {
    populate_incursion_trace(info, depth_samples, boundary_samples, trail_samples,
                             filter, trace, plausible,
                             kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             AircraftModel::filter_type, false, misses, visibility_avg); });

        char date_buffer[32];
        FormatISO8601(date_buffer, aircraft.flight_date_utc_start);

        double max_depth = 0;
        for (const auto &[_, depth] : depth_samples)
            max_depth = std::max(max_depth, depth);

        boost::json::object data = {
            {"id", aircraft.id},
            {"fr_info", aircraft.fr_info},
            {"fr_id", aircraft.fr_id},
            {"in_flock", aircraft.in_flock},
            {"plausible", plausible},
            {"date_start", date_buffer},
            {"max_depth", max_depth},
            {"trace", trace}};

        return data;
    }

    boost::json::object write_terrain(
        const AircraftModel &aircraft,
        const Vignette &info,
        const std::vector<TerrainDistanceSample> &distance_samples,
        const std::vector<EventTrailSample> &trail_samples)
    {
        boost::json::array trace;

        bool plausible = true;
        Averager visibility_avg;
        std::vector<DetectMiss> misses;
        bool kf_valid = AircraftModel::filter_type > 0;
        size_t reconstruction_warmup_samples = 0;

        dispatch_filter(AircraftModel::filter_type, [&](auto &filter)
                        {
    populate_terrain_trace(info, distance_samples, trail_samples,
                           filter, trace, plausible,
                           kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             AircraftModel::filter_type, false, misses, visibility_avg); });

        char date_buffer[32];
        FormatISO8601(date_buffer, aircraft.flight_date_utc_start);

        double min_distance = 0;
        bool has_distance = false;
        for (const auto &sample : distance_samples)
        {
            if (!has_distance)
            {
                min_distance = sample.distance;
                has_distance = true;
            }
            else
                min_distance = std::min(min_distance, sample.distance);
        }

        boost::json::object data = {
            {"id", aircraft.id},
            {"fr_info", aircraft.fr_info},
            {"fr_id", aircraft.fr_id},
            {"in_flock", aircraft.in_flock},
            {"plausible", plausible},
            {"date_start", date_buffer},
            {"distance_min", has_distance ? boost::json::value(min_distance) : boost::json::value(nullptr)},
            {"trace", trace}};

        return data;
    }

} // namespace MultiAircraft::TraceWriter
