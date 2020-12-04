/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2020 The XCSoar Project
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

#ifndef XCSOAR_TRACKING_OGN_CLIENT_HPP
#define XCSOAR_TRACKING_OGN_CLIENT_HPP

#include "thread/Mutex.hxx"
#include "util/Compiler.h"
#include "util/StaticString.hxx"

#include <stdint.h>
#include <boost/asio.hpp>
#include <string>

struct GeoPoint;

namespace OGN {

struct Settings;
class Handler;

class Client {
  Handler *const handler;

  /**
   * Protects #resolver, #socket.
   */
  mutable Mutex mutex;

public:
  explicit Client(boost::asio::io_context &io_context,
                  Handler *_handler=nullptr)
      :handler(_handler), resolver(io_context), socket(io_context), resolving(false) {}
  ~Client() { Close(); }

  void Close();

  void SendTrafficRequest(const ::GeoPoint &location);
  void SetSettings(const Settings& settings);

private:

  void request(const std::string& path);
  void handle_resolve(const boost::system::error_code& err,
                      const boost::asio::ip::tcp::resolver::results_type& endpoints);
  void handle_connect(const boost::system::error_code& err);
  void handle_write_request(const boost::system::error_code& err);
  void handle_read_status_line(const boost::system::error_code& err);
  void handle_read_headers(const boost::system::error_code& err);
  void handle_read_content(const boost::system::error_code& err);
  void handle_traffic_line(const std::string m);

protected:
  boost::asio::ip::tcp::resolver resolver;
  boost::asio::ip::tcp::socket socket;
  bool resolving;
  std::string data;
  std::string content;
  StaticString<64> server;
  StaticString<64> pilot_id;
  unsigned range_km;
};

} /* namespace OGN */

#endif
