#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/times.h>
#endif
#include "timer.h"

/*
  Copyright (c) 2006-2011 Tommi Junttila
  Released under the GNU General Public License version 3.

  This file is part of bliss.

  bliss is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 3
  as published by the Free Software Foundation.

  bliss is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/


namespace bliss {

#ifdef _WIN32
// Windows implementation using GetProcessTimes
Timer::Timer()
{
  reset();
}

void Timer::reset()
{
  FILETIME creationTime, exitTime, kernelTime, userTime;
  if (GetProcessTimes(GetCurrentProcess(), &creationTime, &exitTime, &kernelTime, &userTime)) {
    ULARGE_INTEGER kt, ut;
    kt.LowPart = kernelTime.dwLowDateTime;
    kt.HighPart = kernelTime.dwHighDateTime;
    ut.LowPart = userTime.dwLowDateTime;
    ut.HighPart = userTime.dwHighDateTime;
    // Convert 100-nanosecond intervals to seconds
    start_time = (double)(kt.QuadPart + ut.QuadPart) / 10000000.0;
  } else {
    start_time = 0.0;
  }
}

double Timer::get_duration()
{
  FILETIME creationTime, exitTime, kernelTime, userTime;
  if (GetProcessTimes(GetCurrentProcess(), &creationTime, &exitTime, &kernelTime, &userTime)) {
    ULARGE_INTEGER kt, ut;
    kt.LowPart = kernelTime.dwLowDateTime;
    kt.HighPart = kernelTime.dwHighDateTime;
    ut.LowPart = userTime.dwLowDateTime;
    ut.HighPart = userTime.dwHighDateTime;
    // Convert 100-nanosecond intervals to seconds
    double current_time = (double)(kt.QuadPart + ut.QuadPart) / 10000000.0;
    return current_time - start_time;
  }
  return 0.0;
}

#else
// Unix/Linux implementation using times()
static const double numTicksPerSec = (double)(sysconf(_SC_CLK_TCK));

Timer::Timer()
{
  reset();
}

void Timer::reset()
{
  struct tms clkticks;

  times(&clkticks);
  start_time =
    ((double) clkticks.tms_utime + (double) clkticks.tms_stime) /
    numTicksPerSec;
}


double Timer::get_duration()
{
  struct tms clkticks;

  times(&clkticks);
  double intermediate =
    ((double) clkticks.tms_utime + (double) clkticks.tms_stime) /
    numTicksPerSec;
  return intermediate - start_time;
}
#endif

} // namespace bliss
