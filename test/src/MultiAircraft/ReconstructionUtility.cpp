#include "ReconstructionUtility.hpp"
#include "FlightReconstructionInternal.hpp"
#include <boost/json.hpp>
#include <fstream>
#include <string>
#include <iostream>

namespace json = boost::json;

namespace FlightReconstruction
{
    const DerivState convert_state(const State &state)
    {
        return detail::ConvertState<6, QUATERNION>(state);
    }

    void set_state(State &state,
                   const DerivState &vstate)
    {
        detail::SetState<6, QUATERNION>(state, vstate);
    }

    State get_initial_state_estimate(const double x, const double y, const double z,
                                     const double U,
                                     const double bank,
                                     const double pitch,
                                     const double hdg)
    {
        Eigen::Quaterniond init_quaternion =
            Eigen::AngleAxisd(hdg, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(bank, Eigen::Vector3d::UnitX());
        return State({x, y, z, U, 0.0, 0.0},
                     unscented::UnitQuaternion(init_quaternion));
    }

    const Euler get_euler(const State &state)
    {
        return detail::GetEuler(state);
    }

    void write(const State &state)
    {
        auto &x = convert_state(state);
        for (int i = 0; i < 6; ++i)
        {
            std::cout << x[i] << ", ";
        }
        auto &euler = get_euler(state);
        for (int i = 0; i < 3; ++i)
        {
            std::cout << euler[i] << ", ";
        }
        std::cout << std::endl;
    }

    // Utility function to read file content into a string
    static std::string read_file(const char *filename)
    {
        std::ifstream is(filename);
        if (!is)
        {
            throw std::runtime_error(std::string("Cannot open file: ") + filename);
        }
        std::string s;
        is.seekg(0, std::ios::end);
        s.resize(is.tellg());
        is.seekg(0, std::ios::beg);
        is.read(&s[0], s.size());
        return s;
    }

    std::vector<Measurement> load_encounter(State &initial_state_estimate)
    {
        std::vector<Measurement> measurements;
        try
        {
            std::string filename = "encounter_00000.json";
            // Read the file into a string
            auto const json_data = read_file(filename.c_str());

            // Parse the JSON string into a value
            json::value jv = json::parse(json_data);
            auto trace = jv.get_object().at("aircraft").at(0).at("trace").as_array();
            int count = 0;
            for (auto &line : trace)
            {
                if (count == 2)
                {
                    auto &[x, y, z, U] = measurements[0].data;
                    const double hdg = line.at("hdg").as_double() * DEGTORAD;
                    const double pitch = line.at("pitch").as_double() * DEGTORAD;
                    const double bank = line.at("bank").as_double() * DEGTORAD;
                    initial_state_estimate = get_initial_state_estimate(
                        x.value, y.value, z.value, U.value, bank, pitch, hdg);
                }
                if (count > 0)
                {
                    Measurement measurement;
                    auto &[y, x, z, U] = measurement.data;
                    x.value = line.at("x").as_double();
                    y.value = line.at("y").as_double();
                    z.value = -line.at("alt_gps").as_double();
                    U.value = line.at("v_ias").as_double();
                    measurements.push_back(measurement);
                }
                count++;
            }
        }
        catch (std::exception const &e)
        {
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }
        // json::value jv = json::parse(json_data);
        return measurements;
    }

    int reconstruct_main()
    {
        // Simulation parameters
        const auto DT = 1.0; // seconds

        State initial_state_estimate;
        std::vector<Measurement> measurements =
            load_encounter(initial_state_estimate);

        Filter filter;
        filter.initialise(initial_state_estimate, DT);
        double t = 0.0;

        for (auto &meas : measurements)
        {
            filter.update(meas, DT);
            const auto &est_state = filter.get_state();
            write(est_state);
            t += DT;
        }
        return 0;
    }

}