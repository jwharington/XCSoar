/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2017 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

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

#include "Device/Driver/PI300.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"
#include <stdint.h>
#include <assert.h>

#define PI300_STATUS1_Release                (1<<0) //
#define PI300_STATUS1_Auto                   (1<<1) //
#define PI300_STATUS1_Button                 (1<<2)
#define PI300_STATUS1_WarmStart              (1<<3)
#define PI300_STATUS1_Live                   (1<<4)
#define PI300_STATUS1_BMSlimit               (1<<5) //

#define PI300_STATUS2_LowVolt                (1<<0) //
#define PI300_STATUS2_TempControllerHigh     (1<<1) //
#define PI300_STATUS2_TempMotorHigh          (1<<2)
#define PI300_STATUS2_TempBatteryHigh        (1<<3)
#define PI300_STATUS2_CurrentMotorHigh       (1<<4)
#define PI300_STATUS2_LimitShutdown          (1<<5) //
#define PI300_STATUS2_ReleaseMissing         (1<<6) //
#define PI300_STATUS2_ThrustNonZero          (1<<7) //

class PI300Device final : public AbstractDevice {
  static constexpr uint8_t PI300_WIDTH = 25;
  static constexpr uint8_t PI300_START = 0x79;
  static constexpr uint8_t PI300_END = 0x0d;
  static constexpr uint8_t PI300_CHECKSUM_SEED = 170;

  Port &port;
  int rx_state;
  uint16_t checksum;
  uint8_t rx_buffer[PI300_WIDTH];

  bool parse_byte(const uint8_t ch);
  bool parse_sentence();
  void add_byte(uint8_t ch);
  bool decode_status1();
  bool decode_status2();
  void decode_sentence(PropulsionInfo &pinfo, ElectricPropulsionInfo &info);

public:
  explicit PI300Device(Port &_port):port(_port),
      rx_state(0), checksum(0) {}

public:
  virtual bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;
};

void
PI300Device::add_byte(uint8_t ch)
{
  if (rx_state) {
    if (rx_state < PI300_WIDTH-1)
      checksum += ch;
  } else
    checksum = ch+PI300_CHECKSUM_SEED;

  rx_buffer[rx_state++] = ch;
}

bool
PI300Device::parse_byte(const uint8_t ch)
{
  if (rx_state || (ch == PI300_START)) {
    add_byte(ch);
  } else {
    // not yet started
    return false;
  }

  if (rx_state == PI300_WIDTH) {
    rx_state = 0; // re-start search for start
    return parse_sentence();
  }
  return false;
}

bool
PI300Device::parse_sentence()
{
  // check for end sentence
  if (rx_buffer[PI300_WIDTH-2] != PI300_END) {

    // attempt re-sync
    for (int i=1; i<PI300_WIDTH; ++i) {
      if (rx_buffer[i] == PI300_START) {

        for (int j= i; j<PI300_WIDTH; ++j) {
          add_byte(rx_buffer[j]);
        }
        return false;
      }
    }

    return false;
  }

  // check for checksum error
  const uint8_t checksum_short = (checksum & 0xFF);
  const uint8_t checksum_data = rx_buffer[PI300_WIDTH-1];

  return (checksum_data == checksum_short);
}

void
PI300Device::decode_sentence(PropulsionInfo &pinfo, ElectricPropulsionInfo &info)
{
  info.voltage = 0.01*(rx_buffer[3]+rx_buffer[4]*256.0);
  info.current = (rx_buffer[5]+rx_buffer[6]*256.0-512.0)*1.563/2;
  pinfo.turn_rate = (rx_buffer[7]+rx_buffer[8]*256.0);

  info.temperature_battery = Temperature::FromCelsius(rx_buffer[9]);
  info.temperature_motor = Temperature::FromCelsius(rx_buffer[10]);
  info.temperature_controller = Temperature::FromCelsius(rx_buffer[11]);

  info.capacity = 0.1*(rx_buffer[16]+rx_buffer[17]*256.0);

  // status[0] = rx_buffer[20];
  // status[1] = rx_buffer[21];

  pinfo.throttle = rx_buffer[22];
}

bool
PI300Device::DataReceived(const void *_data, size_t length,
                           NMEAInfo &info)
{
  assert(_data != nullptr);
  assert(length > 0);

  const uint8_t *data = (const uint8_t *)_data;
  const uint8_t *end = data + length;

  bool update = false;

  // data received from device, so it's alive
  info.alive.Update(info.clock);

  while (data<end) {
    if (parse_byte(*data)) {
      update = true;
      decode_sentence(info.propulsion, info.electric_propulsion);
    }
    data++;
  }
  if (update) {
    info.propulsion_available.Update(info.clock);
    info.electric_propulsion_available.Update(info.clock);
  }

  return true;
}


static Device *
PI300CreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new PI300Device(com_port);
}


const DeviceRegister pi300_driver = {
  _T("PI300"),
  _T("PI300"),
  DeviceRegister::NO_TIMEOUT |
  DeviceRegister::RAW_GPS_DATA,
  PI300CreateOnPort,
};


