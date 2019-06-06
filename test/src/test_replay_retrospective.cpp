#include "test_debug.hpp"
#include "TestUtil.hpp"
#include "harness_aircraft.hpp"
#include "Replay/IgcReplay.hpp"
#include "Engine/Navigation/Aircraft.hpp"
#include "NMEA/FlyingState.hpp"
#include "system/ConvertPathName.hpp"
#include "system/FileUtil.hpp"
#include "io/FileLineReader.hpp"
#include "NMEA/Info.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "Engine/Contest/Solvers/Retrospective.hpp"
#include "Waypoint/WaypointReader.hpp"
#include "Waypoint/WaypointReaderBase.hpp"
#include "Operation/Operation.hpp"

#include <fstream>

static bool
test_replay_retrospective()
{
  Directory::Create(Path(_T("output/results")));
  std::ofstream f("output/results/res-sample.txt");

  Waypoints waypoints;
  NullOperationEnvironment operation;
  if (!ok1(ReadWaypointFile(waypoint_file, waypoints,
                            WaypointFactory(WaypointOrigin::NONE),
                            operation))) {
    skip(2, 0, "parsing waypoint file failed");
    return false;
  }

  waypoints.Optimise();

  ok1(!waypoints.IsEmpty());

  Retrospective retro(waypoints);

  retro.search_range = range_threshold;
  retro.angle_tolerance = Angle::Degrees(autopilot_parms.bearing_noise);

  IgcReplay sim(std::make_unique<FileLineReaderA>(replay_file));

  NMEAInfo basic;
  basic.Reset();

  while (sim.Update(basic)) {
    n_samples++;

    if (retro.UpdateSample(basic.location)) {
      std::ofstream g("output/results/res-retro.txt");

      // report task
      auto candidate_list = retro.getNearWaypointList();
      for (auto it = candidate_list.begin(); it != candidate_list.end(); ++it) {
        const WaypointPtr wp = it->waypoint;
        if (wp != nullptr) {
          g << (double)wp->location.longitude.Degrees() << " "
            << (double)wp->location.latitude.Degrees() << " "
            << "\"" << wp->name << "\"\n";
        }
      }
    }

    f << (double)basic.time << " "
      <<  (double)basic.location.longitude.Degrees() << " "
      <<  (double)basic.location.latitude.Degrees() << "\n";
    f.flush();
  };

  double d_ach, d_can;
  retro.CalcDistances(d_ach, d_can);
  printf("# distances %f %f\n", (double)d_ach, (double)d_can);
  printf("# size %d\n", static_cast<int>(retro.getNearWaypointList().size()));

  return true;
}


int main(int argc, char** argv) 
{
  output_skip = 60;

  range_threshold = 15000;
  autopilot_parms.bearing_noise = 25;

  replay_file = Path(_T("test/data/9crx3101.igc"));
  waypoint_file = Path(_T("test/data/benalla9.xcw"));

  if (!ParseArgs(argc,argv)) {
    return 0;
  }

  plan_tests(4);

  ok(test_replay_retrospective(),"replay retrospective",0);

  return exit_status();
}

