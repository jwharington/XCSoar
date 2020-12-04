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

#include "Client.hpp"
#include "Handler.hpp"
#include "io/async/AsioUtil.hpp"
#include <boost/bind.hpp>
#include <string>
#include <regex>
#include "Settings.hpp"
#include "Geo/GeoPoint.hpp"
#include "Geo/GeoVector.hpp"
#include "io/CSVLine.hpp"

void
OGN::Client::Close()
{
  const std::lock_guard<Mutex> lock(mutex);

  if (socket.is_open()) {
    CancelWait(socket);
    socket.close();
  }

  if (resolving) {
    CancelWait(resolver);
    resolving = false;
  }
}

void
OGN::Client::SendTrafficRequest(const ::GeoPoint &location)
{
  Close();

  const ::GeoPoint nw = ::GeoVector(range_km*1000.0, Angle::Degrees(-45)).EndPoint(location);
  const ::GeoPoint se = ::GeoVector(range_km*1000.0, Angle::Degrees(135)).EndPoint(location);
  // &a = all: 1: inactive, 0: active only
  // bounds &b={{ amax }}&c={{ amin }}&d={{ omax }}&e={{ omin }}

  std::string query = std::string("/lxml.php?")
      + "a=0" // 0: active only, 1: active+inactive
      + "&b=" + std::to_string(nw.latitude.Degrees())
      + "&c=" + std::to_string(se.latitude.Degrees())
      + "&d=" + std::to_string(se.longitude.Degrees())
      + "&e=" + std::to_string(nw.longitude.Degrees())
      + "&z=0"; // timezone offset
  request(query);
}


void OGN::Client::request(const std::string& path)
{
  const std::lock_guard<Mutex> lock(mutex);

  // Form the request. We specify the "Connection: close" header so that the
  // server will close the socket after transmitting the response. This will
  // allow us to treat all data up until the EOF as the content.
  data = std::string("GET ") + path + " HTTP/1.0\r\n"
      + "Host: " + server.c_str() + "\r\n"
      + "Accept: */*\r\n"
      + "Connection: close\r\n\r\n";

  // Start an asynchronous resolve to translate the server and service names
  // into a list of endpoints.
  resolving = true;
  resolver.async_resolve(server.c_str(), "http",
                         boost::bind(&Client::handle_resolve, this,
                                     boost::asio::placeholders::error,
                                     boost::asio::placeholders::results));
}

void OGN::Client::handle_resolve(const boost::system::error_code& err,
                                 const boost::asio::ip::tcp::resolver::results_type& endpoints)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    resolving = false;
    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
    boost::asio::async_connect(socket, endpoints,
                               boost::bind(&Client::handle_connect, this,
                                           boost::asio::placeholders::error));
  } else if (handler != nullptr) {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_connect(const boost::system::error_code& err)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    // The connection was successful. Send the request.
    boost::asio::async_write(socket, boost::asio::buffer(data),
                             boost::bind(&Client::handle_write_request, this,
                                         boost::asio::placeholders::error));
  } else if (handler != nullptr) {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_write_request(const boost::system::error_code& err)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    // Read the response status line. The response_ streambuf will
    // automatically grow to accommodate the entire line. The growth may be
    // limited by passing a maximum size to the streambuf constructor.
    boost::asio::async_read_until(socket, boost::asio::dynamic_buffer(data), "\r\n",
                                  boost::bind(&Client::handle_read_status_line, this,
                                              boost::asio::placeholders::error));
  } else if (handler != nullptr) {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_read_status_line(const boost::system::error_code& err)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    // Check that response is OK.
    /*
      std::istream response_stream(&response_);
      std::string http_version;
      response_stream >> http_version;
      unsigned int status_code;
      response_stream >> status_code;
      std::string status_message;
      std::getline(response_stream, status_message);
      if (!response_stream || http_version.substr(0, 5) != "HTTP/")
      {
        std::cout << "Invalid response\n";
        return;
      }
      if (status_code != 200)
      {
        std::cout << "Response returned with status code ";
        std::cout << status_code << "\n";
        return;
      }
      */
    // Read the response headers, which are terminated by a blank line.
    boost::asio::async_read_until(socket, boost::asio::dynamic_buffer(data), "\r\n\r\n",
                                  boost::bind(&Client::handle_read_headers, this,
                                              boost::asio::placeholders::error));
  } else if (handler != nullptr) {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_read_headers(const boost::system::error_code& err)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    // Process the response headers.
    /*
      std::istream response_stream(&response_);
      std::string header;
      while (std::getline(response_stream, header) && header != "\r")
        std::cout << header << "\n";
      std::cout << "\n";
      */

      // Write whatever content we already have to output.
      /*
      if (data.length() > 0)
        std::cout << data;
      */

    // Start reading remaining data until EOF.
    content.clear();
    boost::asio::async_read(socket, boost::asio::dynamic_buffer(content),
                            boost::asio::transfer_at_least(1),
                            boost::bind(&Client::handle_read_content, this,
                                        boost::asio::placeholders::error));
  } else if (handler != nullptr) {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_read_content(const boost::system::error_code& err)
{
  const std::lock_guard<Mutex> lock(mutex);
  if (!err) {
    // Continue reading remaining data until EOF.
    boost::asio::async_read(socket, boost::asio::dynamic_buffer(content),
                            boost::asio::transfer_at_least(1),
                            boost::bind(&Client::handle_read_content, this,
                                        boost::asio::placeholders::error));
  } else if (handler == nullptr) {
    return;
  } else if (err == boost::asio::error::eof) {
    std::regex lines_regex("<m a=\"(.*)\"/>");
    auto words_begin = std::sregex_iterator(content.begin(), content.end(), lines_regex);
    auto words_end = std::sregex_iterator();
    for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
      if (i->size()==2) {
        handle_traffic_line((*i)[1]);
      }
    }
  } else {
    handler->OnOGNError(std::make_exception_ptr(boost::system::system_error(err)));
  }
}

void OGN::Client::handle_traffic_line(const std::string m)
{
  CSVLine line(m.c_str());
  char buf[64];
  double lat = line.Read(0.0);
  double lon = line.Read(0.0);
  line.Read(buf,64); // comp number
  std::string cid(buf);

  line.Read(buf,64); // reg number
  // std::string reg(buf);

  int alt = line.Read(0);
  line.Read(buf,64); // time
  unsigned hour, minute, second;
  unsigned time = 0;
  if (sscanf(buf, "%02u:%02u:%02u", &hour, &minute, &second) == 3) {
    time = second + 60*(minute+60*hour);
  }
  unsigned ddf = line.Read(0); (void)ddf;
  double track_deg = line.Read(0.0); (void)track_deg;
  double speed_kph = line.Read(0.0); (void)speed_kph;
  double vz = line.Read(0.0); (void)vz;
  unsigned typ = line.Read(0); (void)typ;
  line.Read(buf,64); // recorder
  unsigned fid = line.ReadHex(0);
  line.Read(buf,64); // crc
  if (fid // flight id is not do-not-track
      && (0!= std::strncmp(cid.c_str(), pilot_id.c_str(), 64)) // don't include self
      && (alt>0) // altitude valid
      ) {
    handler->OnOGNTraffic(fid, cid.c_str(), time, ::GeoPoint(Angle::Degrees(lon),Angle::Degrees(lat)), alt);
  }
}

void OGN::Client::SetSettings(const Settings& settings)
{
  server = settings.server;
  pilot_id = settings.pilot_id;
  range_km = settings.range_km;
}
