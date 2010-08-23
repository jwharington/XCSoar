/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "InfoBoxes/Content/Factory.hpp"

#include "InfoBoxes/Content/Base.hpp"
#include "InfoBoxes/Content/Alternate.hpp"
#include "InfoBoxes/Content/Altitude.hpp"
#include "InfoBoxes/Content/Direction.hpp"
#include "InfoBoxes/Content/Glide.hpp"
#include "InfoBoxes/Content/Other.hpp"
#include "InfoBoxes/Content/Speed.hpp"
#include "InfoBoxes/Content/Task.hpp"
#include "InfoBoxes/Content/Team.hpp"
#include "InfoBoxes/Content/Thermal.hpp"
#include "InfoBoxes/Content/Time.hpp"
#include "InfoBoxes/Content/Weather.hpp"

#include "Language.hpp"

#include <stddef.h>

// Groups:
//   Altitude 0,1,20,33
//   Aircraft info 3,6,23,32,37,47,54
//   LD 4,5,19,38,53,66
//   Vario 2,7,8,9,21,22,24,44
//   Wind 25,26,48,49,50
//   MacCready 10,34,35,43
//   Nav 11,12,13,15,16,17,18,27,28,29,30,31
//   Waypoint 14,36,39,40,41,42,45,46
const InfoBoxFactory::InfoBoxMetaData InfoBoxFactory::MetaData[NUM_TYPES] = {
  // 0
  {
    N_("Height GPS"),
    N_("H GPS"),
    1, 33,
  },

  // 1
  {
    N_("Height AGL"),
    N_("H AGL"),
    20, 0,
  },

  // 2
  {
    N_("Thermal last 30 sec"),
    N_("TC 30s"),
    7, 44,
  },

  // 3
  {
    N_("Bearing"),
    N_("Bearing"),
    6, 54,
  },

  // 4
  {
    N_("L/D instantaneous"),
    N_("L/D Inst"),
    5, 38,
  },

  // 5
  {
    N_("L/D cruise"),
    N_("L/D Cru"),
    19, 4,
  },

  // 6
  {
    N_("Speed ground"),
    N_("V Gnd"),
    23, 3,
  },

  // 7
  {
    N_("Last Thermal Average"),
    N_("TL Avg"),
    8, 2,
  },

  // 8
  {
    N_("Last Thermal Gain"),
    N_("TL Gain"),
    9, 7,
  },

  // 9
  {
    N_("Last Thermal Time"),
    N_("TL Time"),
    21, 8,
  },

  // 10
  {
    N_("MacCready Setting"),
    N_("MacCready"),
    34, 43,
  },

  // 11
  {
    N_("Next Distance"),
    N_("WP Dist"),
    12, 31,
  },

  // 12
  {
    N_("Next Altitude Difference"),
    N_("WP AltD"),
    13, 11,
  },

  // 13
  {
    N_("Next Altitude Required"),
    N_("WP AltR"),
    15, 12,
  },

  // 14
  {
    N_("Next Waypoint"),
    N_("Next"),
    36, 46,
  },

  // 15
  {
    N_("Final Altitude Difference"),
    N_("Fin AltD"),
    16, 13,
  },

  // 16
  {
    N_("Final Altitude Required"),
    N_("Fin AltR"),
    17, 15,
  },

  // 17
  {
    N_("Speed Task Average"),
    N_("V Task Av"),
    18, 16,
  },

  // 18
  {
    N_("Final Distance"),
    N_("Fin Dis"),
    27, 17,
  },

  // 19
  {
    N_("Final LD"),
    N_("Fin LD"),
    38, 5,
  },

  // 20
  {
    N_("Terrain Elevation"),
    N_("H GND"),
    33, 1,
  },

  // 21
  {
    N_("Thermal Average"),
    N_("TC Avg"),
    22, 9,
  },

  // 22
  {
    N_("Thermal Gain"),
    N_("TC Gain"),
    24, 21,
  },

  // 23
  {
    N_("Track"),
    N_("Track"),
    32, 6,
  },

  // 24
  {
    N_("Vario"),
    N_("Vario"),
    44, 22,
  },

  // 25
  {
    N_("Wind Speed"),
    N_("Wind V"),
    26, 50,
  },

  // 26
  {
    N_("Wind Bearing"),
    N_("Wind B"),
    48, 25,
  },

  // 27
  {
    N_("AA Time"),
    N_("AA Time"),
    28, 18,
  },

  // 28
  {
    N_("AA Distance Max"),
    N_("AA Dmax"),
    29, 27,
  },

  // 29
  {
    N_("AA Distance Min"),
    N_("AA Dmin"),
    30, 28,
  },

  // 30
  {
    N_("AA Speed Max"),
    N_("AA Vmax"),
    31, 29,
  },

  // 31
  {
    N_("AA Speed Min"),
    N_("AA Vmin"),
    51, 30,
  },

  // 32
  {
    N_("Airspeed IAS"),
    N_("V IAS"),
    37, 23,
  },

  // 33
  {
    N_("Pressure Altitude"),
    N_("H Baro"),
    0, 20,
  },

  // 34
  {
    N_("Speed MacCready"),
    N_("V Mc"),
    35, 10,
  },

  // 35
  {
    N_("Percentage climb"),
    N_("% Climb"),
    43, 34,
  },

  // 36
  {
    N_("Time of flight"),
    N_("Time flt"),
    39, 14,
  },

  // 37
  {
    N_("G load"),
    N_("G"),
    47, 32,
  },

  // 38
  {
    N_("Next LD"),
    N_("WP LD"),
    53, 19,
  },

  // 39
  {
    N_("Time local"),
    N_("Time loc"),
    40, 36,
  },

  // 40
  {
    N_("Time UTC"),
    N_("Time UTC"),
    41, 39,
  },

  // 41
  {
    N_("Task Time To Go"),
    N_("Fin ETE"),
    42, 40,
  },

  // 42
  {
    N_("Next Time To Go"),
    N_("WP ETE"),
    45, 41,
  },

  // 43
  {
    N_("Speed Dolphin"),
    N_("V Opt"),
    10, 35,
  },

  // 44
  {
    N_("Netto Vario"),
    N_("Netto"),
    2, 24,
  },

  // 45
  {
    N_("Task Arrival Time"),
    N_("Fin ETA"),
    46, 42,
  },

  // 46
  {
    N_("Next Arrival Time"),
    N_("WP ETA"),
    14, 45,
  },

  // 47
  {
    N_("Bearing Difference"),
    N_("Brng D"),
    54, 37,
  },

  // 48
  {
    N_("Outside Air Temperature"),
    N_("OAT"),
    49, 26,
  },

  // 49
  {
    N_("Relative Humidity"),
    N_("RelHum"),
    50, 48,
  },

  // 50
  {
    N_("Forecast Temperature"),
    N_("MaxTemp"),
    49, 25,
  },

  // 51
  {
    N_("AA Distance Tgt"),
    N_("AA Dtgt"),
    52, 31,
  },

  // 52
  {
    N_("AA Speed Tgt"),
    N_("AA Vtgt"),
    11, 51,
  },

  // 53
  {
    N_("L/D vario"),
    N_("L/D vario"),
    4, 38,
  },

  // 54
  {
    N_("Airspeed TAS"),
    N_("V TAS"),
    3, 47,
  },

  // 55
  {
    N_("Own Team Code"),
    N_("TeamCode"),
    56, 54,
  },

  // 56
  {
    N_("Team Bearing"),
    N_("Tm Brng"),
    57, 55,
  },

  // 57
  {
    N_("Team Bearing Diff"),
    N_("Team Bd"),
    58, 56,
  },

  // 58
  {
    N_("Team Range"),
    N_("Team Dis"),
    55, 57,
  },

  // 59
  {
    N_("Speed Task Instantaneous"),
    N_("V Tsk Ins"),
    18, 16,
  },

  // 60
  {
    N_("Distance Home"),
    N_("Home Dis"),
    18, 16,
  },

  // 61
  {
    N_("Speed Task Achieved"),
    N_("V Tsk Ach"),
    18, 16,
  },

  // 62
  {
    N_("AA Delta Time"),
    N_("AA dT"),
    28, 18,
  },

  // 63
  {
    N_("Thermal All"),
    N_("TC All"),
    8, 2,
  },

  // 64
  {
    N_("Distance Vario"),
    N_("D Vario"),
    8, 2,
  },

  // 65
  {
#ifndef GNAV
    N_("Battery Percent"),
#else
    N_("Battery Voltage"),
#endif
    N_("Battery"),
    49, 26,
  },

  // 66  VENTA-ADDON added Final GR
  // VENTA-TODO: fix those 38,5 numbers to point correctly menu items
  {
    N_("Final GR"),
    N_("Fin GR"),
    38, 5,
  },

  // 67 VENTA3-ADDON Alternate1 destinations infoboxes  TODO> fix 36 46 to something correct
  {
    N_("Alternate1 GR"),
    N_("Altern 1"),
    36, 46,
  },

  // 68 Alternate 2
  {
    N_("Alternate2 GR"),
    N_("Altern 2"),
    36, 46,
  },

  // 69 BestAlternate aka BestLanding
  {
    N_("Best Alternate"),
    N_("BestAltn"),
    36, 46,
  },

  // 70
  {
    N_("QFE GPS"),
    N_("QFE GPS"),
    1, 33,
  },

  // 71 TODO FIX those 19,4 values
  {
    N_("L/D Average"),
    N_("L/D Avg"),
    19, 4,
  },

  // 72 //
  {
    N_("Experimental1"),
    N_("Exp1"),
    8, 2,
  },

  // 73 //
  {
    N_("Online Contest Distance"),
    N_("OLC"),
    8, 2,
  },

  // 74 //
  {
    N_("Experimental2"),
    N_("Exp2"),
    8, 2,
  },
};

InfoBoxContent*
InfoBoxFactory::Create(unsigned InfoBoxType)
{
  switch (InfoBoxType) {
  case 0:
    return new InfoBoxContentAltitudeGPS();
  case 1:
    return new InfoBoxContentAltitudeAGL();
  case 2:
    return new InfoBoxContentThermal30s();
  case 3:
    return new InfoBoxContentBearing();
  case 4:
    return new InfoBoxContentLDInstant();
  case 5:
    return new InfoBoxContentLDCruise();
  case 6:
    return new InfoBoxContentSpeedGround();
  case 7:
    return new InfoBoxContentThermalLastAvg();
  case 8:
    return new InfoBoxContentThermalLastGain();
  case 9:
    return new InfoBoxContentThermalLastTime();
  case 10:
    return new InfoBoxContentMacCready();
  case 11:
    return new InfoBoxContentNextDistance();
  case 12:
    return new InfoBoxContentNextAltitudeDiff();
  case 13:
    return new InfoBoxContentNextAltitudeRequire();
  case 14:
    return new InfoBoxContentNextWaypoint();
  case 15:
    return new InfoBoxContentFinalAltitudeDiff();
  case 16:
    return new InfoBoxContentFinalAltitudeRequire();
  case 17:
    return new InfoBoxContentTaskSpeed();
  case 18:
    return new InfoBoxContentFinalDistance();
  case 19:
    return new InfoBoxContentFinalLD();
  case 20:
    return new InfoBoxContentTerrainHeight();
  case 21:
    return new InfoBoxContentThermalAvg();
  case 22:
    return new InfoBoxContentThermalGain();
  case 23:
    return new InfoBoxContentTrack();
  case 24:
    return new InfoBoxContentVario();
  case 25:
    return new InfoBoxContentWindSpeed();
  case 26:
    return new InfoBoxContentWindBearing();
  case 27:
    return new InfoBoxContentTaskAATime();
  case 28:
    return new InfoBoxContentTaskAADistanceMax();
  case 29:
    return new InfoBoxContentTaskAADistanceMin();
  case 30:
    return new InfoBoxContentTaskAASpeedMax();
  case 31:
    return new InfoBoxContentTaskAASpeedMin();
  case 32:
    return new InfoBoxContentSpeedIndicated();
  case 33:
    return new InfoBoxContentAltitudeBaro();
  case 34:
    return new InfoBoxContentSpeedMacCready();
  case 35:
    return new InfoBoxContentThermalRatio();
  case 36:
    return new InfoBoxContentTimeFlight();
  case 37:
    return new InfoBoxContentGLoad();
  case 38:
    return new InfoBoxContentNextLD();
  case 39:
    return new InfoBoxContentTimeLocal();
  case 40:
    return new InfoBoxContentTimeUTC();
  case 41:
    return new InfoBoxContentFinalETE();
  case 42:
    return new InfoBoxContentNextETE();
  case 43:
    return new InfoBoxContentSpeedDolphin();
  case 44:
    return new InfoBoxContentVarioNetto();
  case 45:
    return new InfoBoxContentFinalETA();
  case 46:
    return new InfoBoxContentNextETA();
  case 47:
    return new InfoBoxContentBearingDiff();
  case 48:
    return new InfoBoxContentTemperature();
  case 49:
    return new InfoBoxContentHumidity();
  case 50:
    return new InfoBoxContentTemperatureForecast();
  case 51:
    return new InfoBoxContentTaskAADistance();
  case 52:
    return new InfoBoxContentTaskAASpeed();
  case 53:
    return new InfoBoxContentLDVario();
  case 54:
    return new InfoBoxContentSpeed();
  case 55:
    return new InfoBoxContentTeamCode();
  case 56:
    return new InfoBoxContentTeamBearing();
  case 57:
    return new InfoBoxContentTeamBearingDiff();
  case 58:
    return new InfoBoxContentTeamDistance();
  case 59:
    return new InfoBoxContentTaskSpeedInstant();
  case 60:
    return new InfoBoxContentHomeDistance();
  case 61:
    return new InfoBoxContentTaskSpeedAchieved();
  case 62:
    return new InfoBoxContentTaskAATimeDelta();
  case 63:
    return new InfoBoxContentThermalAllAvg();
  case 64:
    return new InfoBoxContentVarioDistance();
  case 65:
    return new InfoBoxContentBattery();
  case 66:
    return new InfoBoxContentFinalGR();
  case 67:
    return new InfoBoxContentAlternate1();
  case 68:
    return new InfoBoxContentAlternate2();
  case 69:
    return new InfoBoxContentAlternateBest();
  case 70:
    return new InfoBoxContentAltitudeQFE();
  case 71:
    return new InfoBoxContentLDAvg();
  case 72:
    return new InfoBoxContentExperimental1();
  case 73:
    return new InfoBoxContentOLC();
  case 74:
    return new InfoBoxContentExperimental2();
  }

  return NULL;
}

const TCHAR*
InfoBoxFactory::GetHelpText(unsigned InfoBoxType)
{
  switch (InfoBoxType) {
  case 0:
    return _("[Height GPS]\r\nThis is the height above mean sea level reported by the GPS.\r\nTouchscreen/PC only: in simulation mode, this value is adjustable with the up/down arrow keys and the right/left arrow keys also cause the glider to turn.");
  case 1:
    return _("[Height AGL]\r\nThis is the navigation altitude minus the terrain height obtained from the terrain file.  The value is coloured red when the glider is below the terrain safety clearance height.");
  case 2:
    return _("[Thermal last 30 sec]\r\nA 30 second rolling average climb rate based of the reported GPS altitude, or vario if available.");
  case 3:
    return _("[Bearing]\r\nTrue bearing of the next waypoint.  For AAT tasks, this is the true bearing to the target within the AAT sector.");
  case 4:
    return _("[L/D instantaneous]\r\nInstantaneous glide ratio, given by the ground speed divided by the vertical speed (GPS speed) over the last 20 seconds.  Negative values indicate climbing cruise. If the vertical speed is close to zero, the displayed value is '---'.");
  case 5:
    return _("[L/D cruise]\r\nThe distance from the top of the last thermal, divided by the altitude lost since the top of the last thermal.  Negative values indicate climbing cruise (height gain since leaving the last thermal).  If the vertical speed is close to zero, the displayed value is '---'.");
  case 6:
    return _("[Speed ground]\r\nGround speed measured by the GPS.  If this infobox is active in simulation mode, pressing the up and down arrows adjusts the speed, and left and right turn the glider.");
  case 7:
    return _("[Last Thermal Average]\r\nTotal altitude gain/loss in the last thermal divided by the time spent circling.");
  case 8:
    return _("[Last Thermal Gain]\r\nTotal altitude gain/loss in the last thermal.");
  case 9:
    return _("[Last Thermal Time]\r\nTime spent circling in the last thermal.");
  case 10:
    return _("[MacCready Setting]\r\nThe current MacCready setting.  This infobox also shows whether MacCready is manual or auto.  (Touchscreen/PC only) Also used to adjust the MacCready Setting if the infobox is active, by using the up/down cursor keys.");
  case 11:
    return _("[Next Distance]\r\nThe distance to the currently selected waypoint. For AAT tasks, this is the distance to the target within the AAT sector.");
  case 12:
    return _("[Next Altitude Difference]\r\nArrival altitude at the next waypoint relative to the safety arrival altitude.");
  case 13:
    return _("[Next Altitude Required]\r\nAltitude required to reach the next turn point.");
  case 14:
    return _("[Next Waypoint]\r\n The name of the currently selected turn point.  When this infobox is active, using the up/down cursor keys selects the next/previous waypoint in the task.  (Touchscreen/PC only) Pressing the enter cursor key brings up the waypoint details.");
  case 15:
    return _("[Final Altitude Difference]\r\nArrival altitude at the final task turn point relative to the safety arrival altitude.");
  case 16:
    return _("[Final Altitude Required]\r\nAltitude required to finish the task.");
  case 17:
    return _("[Speed Task Average]\r\nAverage cross country speed while on current task, compensated for altitude.");
  case 18:
    return _("[Final Distance]\r\nDistance to finish around remaining turn points.");
  case 19:
    return _("[Final L/D]\r\nThe required glide ratio to finish the task, given by the distance to go divided by the height required to arrive at the safety arrival altitude.  Negative values indicate a climb is necessary to finish. If the height required is close to zero, the displayed value is '---'.  Note that this calculation may be optimistic because it reduces the height required to finish by the excess energy height of the glider if its true airspeed is greater than the MacCready and best LD speeds.");
  case 20:
    return _("[Terrain Elevation]\r\nThis is the elevation of the terrain above mean sea level, obtained from the terrain file at the current GPS location.");
  case 21:
    return _("[Thermal Average]\r\nAltitude gained/lost in the current thermal, divided by time spent thermaling.");
  case 22:
    return _("[Thermal Gain]\r\nThe altitude gained/lost in the current thermal.");
  case 23:
    return _("[Track]\r\nMagnetic track reported by the GPS.  (Touchscreen/PC only) If this infobox is active in simulation mode, pressing the up and down  arrows adjusts the track.");
  case 24:
    return _("[Vario]\r\nInstantaneous vertical speed, as reported by the GPS, or the intelligent vario total energy vario value if connected to one.");
  case 25:
    return _("[Wind Speed]\r\nWind speed estimated by XCSoar.  (Touchscreen/PC only) Manual adjustment is possible by pressing the up/down cursor keys to adjust magnitude and left/right cursor keys to adjust bearing when the infobox is active.  Pressing the enter cursor key saves the wind value as the initial value when XCSoar next starts.");
  case 26:
    return _("[Wind Bearing]\r\nWind bearing estimated by XCSoar.  (Touchscreen/PC only) Manual adjustment is possible by pressing the up/down cursor keys to adjust bearing when the infobox is active.");
  case 27:
    return _("[AA Time]\r\nAssigned Area Task time remaining.  Goes red when time remaining has expired.");
  case 28:
    return _("[AA Distance Max]\r\nAssigned Area Task maximum distance possible for remainder of task.");
  case 29:
    return _("[AA Distance Min]\r\nAssigned Area Task minimum distance possible for remainder of task.");
  case 30:
    return _("[AA Speed Max]\r\nAssigned Area Task average speed achievable if flying maximum possible distance remaining in minimum AAT time.");
  case 31:
    return _("[AA Speed Min]\r\nAssigned Area Task average speed achievable if flying minimum possible distance remaining in minimum AAT time.");
  case 32:
    return _("[Airspeed IAS]\r\nIndicated Airspeed reported by a supported external intelligent vario.");
  case 33:
    return _("[Pressure Altitude]\r\nThis is the barometric altitude obtained from a GPS equipped with pressure sensor, or a supported external intelligent vario.");
  case 34:
    return _("[Speed MacCready]\r\nThe MacCready speed-to-fly for optimal flight to the next waypoint. In cruise flight mode, this speed-to-fly is calculated for maintaining altitude.  In final glide mode, this speed-to-fly is calculated for descent.");
  case 35:
    return _("[Percentage climb]\r\nPercentage of time spent in climb mode.  These statistics are reset upon starting the task.");
  case 36:
    return _("[Time of flight]\r\nTime elapsed since takeoff was detected.");
  case 37:
    return _("[G load]\r\nMagnitude of G loading reported by a supported external intelligent vario.  This value is negative for pitch-down manoeuvres.");
  case 38:
    return _("[Next L/D]\r\nThe required glide ratio to reach the next waypoint, given by the distance to next waypoint divided by the height required to arrive at the safety arrival altitude.  Negative values indicate a climb is necessary to reach the waypoint.  If the height required is close to zero, the displayed value is '---'.    Note that this calculation may be optimistic because it reduces the height required to reach the waypoint by the excess energy height of the glider if its true airspeed is greater than the MacCready and best LD speeds.");
  case 39:
    return _("[Time local]\r\nGPS time expressed in local time zone.");
  case 40:
    return _("[Time UTC]\r\nGPS time expressed in UTC.");
  case 41:
    return _("[Task Time To Go]\r\nEstimated time required to complete task, assuming performance of ideal MacCready cruise/climb cycle.");
  case 42:
    return _("[Next Time To Go]\r\nEstimated time required to reach next waypoint, assuming performance of ideal MacCready cruise/climb cycle.");
  case 43:
    return _("[Speed Dolphin]\r\nThe instantaneous MacCready speed-to-fly, making use of Netto vario calculations to determine dolphin cruise speed in the glider's current bearing.  In cruise flight mode, this speed-to-fly is calculated for maintaining altitude.  In final glide mode, this speed-to-fly is calculated for descent.  In climb mode, this switches to the speed for minimum sink at the current load factor (if an accelerometer is connected).  When Block mode speed to fly is selected, this infobox displays the MacCready speed.");
  case 44:
    return _("[Netto Vario]\r\nInstantaneous vertical speed of air-mass, equal to vario value less the glider's estimated sink rate.  Best used if airspeed, accelerometers and vario are connected, otherwise calculations are based on GPS measurements and wind estimates.");
  case 45:
    return _("[Task Arrival Time]\r\nEstimated arrival local time at task completion, assuming performance of ideal MacCready cruise/climb cycle.");
  case 46:
    return _("[Next Arrival Time]\r\nEstimated arrival local time at next waypoint, assuming performance of ideal MacCready cruise/climb cycle.");
  case 47:
    return _("[Bearing Difference]\r\nThe difference between the glider's track bearing, to the bearing of the next waypoint, or for AAT tasks, to the bearing to the target within the AAT sector.  GPS navigation is based on the track bearing across the ground, and this track bearing may differ from the glider's heading when there is wind present.  Chevrons point to the direction the glider needs to alter course to correct the bearing difference, that is, so that the glider's course made good is pointing directly at the next waypoint.  This bearing takes into account the curvature of the Earth.");
  case 48:
    return _("[Outside Air Temperature]\r\nOutside air temperature measured by a probe if supported by a connected intelligent variometer.");
  case 49:
    return _("[Relative Humidity]\r\nRelative humidity of the air in percent as measured by a probe if supported by a connected intelligent variometer.");
  case 50:
    return _("[Forecast Temperature]\r\nForecast temperature of the ground at the home airfield, used in estimating convection height and cloud base in conjunction with outside air temperature and relative humidity probe.  (Touchscreen/PC only) Pressing the up/down cursor keys adjusts this forecast temperature.");
  case 51:
    return _("[AA Distance Tgt]\r\nAssigned Area Task distance around target points for remainder of task.");
  case 52:
    return _("[AA Speed Tgt]\r\nAssigned Area Task average speed achievable around target points remaining in minimum AAT time.");
  case 53:
    return _("[L/D vario]\r\nInstantaneous glide ratio, given by the indicated airspeed divided by the total energy vertical speed, when connected to an intelligent variometer.  Negative values indicate climbing cruise. If the total energy vario speed is close to zero, the displayed value is '---'.");
  case 54:
    return _("[Airspeed TAS]\r\nTrue Airspeed reported by a supported external intelligent vario.");
  case 55:
    return _("[Own Team Code]\r\nThe current Team code for this aircraft.  Use this to report to other team members.");
  case 56:
    return _("[Team Bearing]\r\nThe bearing to the team aircraft location at the last team code report.");
  case 57:
    return _("[Team Bearing Diff]\r\nThe relative bearing to the team aircraft location at the last reported team code.");
  case 58:
    return _("[Team range]\r\nThe range to the team aircraft location at the last reported team code.");
  case 59:
    return _("[Speed Task Instantaneous]\r\nInstantaneous cross country speed while on current task, compensated for altitude.");
  case 60:
    return _("[Distance Home]\r\nDistance to home waypoint (if defined).");
  case 61:
    return _("[Speed Task Achieved]\r\nAchieved cross country speed while on current task, compensated for altitude.");
  case 62:
    return _("[AA Delta Time]\r\nDifference between estimated task time and AAT minimum time.  Colored red if negative (expected arrival too early), or blue if in sector and can turn now with estimated arrival time greater than AAT time plus 5 minutes.");
  case 63:
    return _("[Thermal All]\r\nTime-average climb rate in all thermals.");
  case 65:
    return _("[Battery]\r\nSupply battery voltage for Altair systems, for PDA systems, this gives the percent battery capacity available.");
  case 66:
    return _("[Final GR]\r\nGeometric gradient to the arrival height above the final waypoint.  This is not adjusted for total energy.");
  case 67:
  case 68:
    return _("[Alternate GR]\r\nGeometric gradient to the arrival height above the selected alternate waypoint.  This is not adjusted for total energy.\r\nOver 200 nothing is shown, between 100 and 200 an integer number is shown, between 1 and 99 a decimal value is shown.\r\nThe Alternate value is RED if:\r\n- above the best LD or unreachable (terrain, wind, arrival altitude is less than 100m OVER safety altitude)\r\nThe bottom line is changing every 3 seconds showing distance or arrival altitude over safety.\r\n All values are calculated in real-time\r\nPRESS the ENTER key to make the relative waypoint info page come up: here you can perform actions on the alternate such as goto, insert in task etc.");
  case 69:
    return _("[BestAlternate]\r\nAutomatic search for the best landing option available. Top line is name of landpoint, middle is LD required, bottom line hold distance and arrival altitude over safety, swapped every few seconds. Please read the full manual document about the BestAlternate and how you can customise its behaviour. This is still experimental so be careful. If you select BestAlternate by clicking on the infobox and the PRESS ENTER either with a real key or virtual key then the waypoint detail page will come up and you will be able to perform actions on this Alternate such as goto, inserti in task etc.");
  case 70:
    return _("[QFE]\r\nAutomatic QFE. This altitude value is constantly reset to 0 on ground BEFORE taking off. After takeoff, it is no more reset automatically even if on ground. During flight you can change QFE with up and down keys. Bottom line shows QNH altitude. \r\nChanging QFE does not affect QNH altitude.");
  case 71:
    return _("[L/D average]\r\nThe distance made in the configured period of time , divided by the altitude lost since then.  Negative values are shown as ^^^ and indicate climbing cruise (height gain). Over 200 of LD the value is shown as +++ .\r\nYou can configure the period of averaging in the Special config menu. Suggested values for this configuration are 60, 90 or 120: lower values will be closed to LD INST, and higher values will be closed to LD Cruise. \r\nNotice that the distance is NOT the straight line between you old and current position: it's exactly the distance you have made even in a zigzag glide.\r\nThis value is not calculated while circling. ");
  }

  return NULL;
}
