/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#include "AndroidPort.hpp"
#include "Android/PortBridge.hpp"

#include <assert.h>

AndroidPort::AndroidPort(Handler &_handler, PortBridge *_bridge)
  :Port(_handler), bridge(_bridge)
{
}

AndroidPort::~AndroidPort()
{
  StopRxThread();

  delete bridge;
}

void
AndroidPort::Flush()
{
  bridge->flush(Java::GetEnv());
}

bool
AndroidPort::Drain()
{
  return bridge != NULL && bridge->drain(Java::GetEnv());
}

void
AndroidPort::Run()
{
  assert(bridge != NULL);

  SetRxTimeout(500);

  JNIEnv *const env = Java::GetEnv();

  char buffer[1024];
  while (!CheckStopped()) {
    int nbytes = bridge->read(env, buffer, sizeof(buffer));
    if (nbytes > 0)
      handler.DataReceived(buffer, nbytes);
  }
}

unsigned
AndroidPort::GetBaudrate() const
{
  return bridge != NULL
    ? bridge->getBaudRate(Java::GetEnv())
    : 0;
}

bool
AndroidPort::SetBaudrate(unsigned baud_rate)
{
  return bridge != NULL &&
    bridge->setBaudRate(Java::GetEnv(), baud_rate);
}

size_t
AndroidPort::Write(const void *data, size_t length)
{
  if (bridge == NULL)
    return 0;

  JNIEnv *env = Java::GetEnv();
  int nbytes = bridge->write(env, data, length);
  return nbytes > 0
    ? (size_t)nbytes
    : 0;
}

bool
AndroidPort::StopRxThread()
{
  // Make sure the thread isn't terminating itself
  assert(!Thread::IsInside());

  if (bridge == NULL)
    return false;

  // If the thread is not running, cancel the rest of the function
  if (!Thread::IsDefined())
    return true;

  BeginStop();

  Thread::Join();

  return true;
}

bool
AndroidPort::StartRxThread()
{
  // Make sure the thread isn't starting itself
  assert(!Thread::IsInside());

  // Make sure the port was opened correctly
  if (bridge == NULL)
    return false;

  // Start the receive thread
  StoppableThread::Start();
  return true;
}

bool
AndroidPort::SetRxTimeout(unsigned Timeout)
{
  if (bridge == NULL)
    return false;

  bridge->setReadTimeout(Java::GetEnv(), Timeout);
  return true;
}

int
AndroidPort::Read(void *buffer, size_t length)
{
  JNIEnv *env = Java::GetEnv();
  return bridge->read(env, buffer, length);
}

Port::WaitResult
AndroidPort::WaitRead(unsigned timeout_ms)
{
  return (Port::WaitResult)bridge->waitRead(Java::GetEnv(), timeout_ms);
}
